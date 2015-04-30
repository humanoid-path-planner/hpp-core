//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/model/configuration.hh>
#include <hpp/core/path-optimization/gradient-based.hh>
#include <hpp/core/path-optimization/path-length.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method-straight.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      GradientBasedPtr_t GradientBased::create
      (const Problem& problem)
      {
	GradientBased* ptr = new GradientBased (problem);
	GradientBasedPtr_t shPtr (ptr);
	return shPtr;
      }

      void GradientBased::cost (const CostPtr_t& cost)
      {
	cost_ = cost;
	// Approximation of the Hessian updated according to BFGS formula
	I_.setIdentity (cost_->inputDerivativeSize (),
			cost_->inputDerivativeSize ());
	H_.resize (cost_->inputDerivativeSize (),
		   cost_->inputDerivativeSize ());
	cost->hessianInverse (H_);
	alpha_ = .2;
      }

      PathVectorPtr_t GradientBased::optimize (const PathVectorPtr_t& path)
      {
	if (!cost_ || cost_->inputSize () !=
	    ((size_type)path->numberPaths () - 1) * path->outputSize () ||
	    cost_->inputDerivativeSize () !=
	    ((size_type)path->numberPaths () - 1) *
	    path->outputDerivativeSize ()) {
	  hppDout (info, "creating cost");
	  cost (PathLength::create (distance_, path));
	}

	// Store first and last way points.
	PathPtr_t first = path->pathAtRank (0);
	initial_ = (*first) (path->timeRange ().first);
	std::size_t nbPaths = path->numberPaths ();
	PathPtr_t last = path->pathAtRank (nbPaths-1);
	end_ = (*last) (last->timeRange ().second);

	value_type epsilon = 1e-6;
	value_type alphaMax = .9;

	vector_t x1; x1.resize (cost_->inputSize ());
	matrix_t grad1; grad1.resize (1, cost_->inputDerivativeSize ());
	pathToVector (path, x1);
	vector_t x0 = x1;
	cost_->jacobian (grad1, x1);
	matrix_t grad0 = grad1;
	vector_t cost (1);
	// (*cost_) (cost, x1);
	// hppDout (info, "cost = " << cost [0]);
	hppDout (info, "gradient = " << grad1.transpose ());
	bool stopCondition = (grad1.squaredNorm () <= epsilon);
	std::size_t iter = 0;
	while (!stopCondition) {
	  vector_t p = -H_*(grad0.transpose ());
	  // Project p on constraints

	  vector_t s = alpha_*p;
	  integrate (x0, s, x1);
	  // test path and add constraints if necessary

	  cost_->jacobian (grad1, x1);
	  // (*cost_) (cost, x1);
	  // hppDout (info, "cost = " << cost [0]);
	  hppDout (info, "gradient = " << grad1.transpose ());
	  bool success = (grad1.squaredNorm () <= epsilon);
	  stopCondition = success || iter >= 20;
	  if (stopCondition) {
	    break;
	  }
	  Eigen::Matrix <value_type, 1, Eigen::Dynamic> yT = (grad1 - grad0);
	  matrix_t syT (s * yT);
	  value_type rho = 1./syT.trace ();
	  matrix_t G (I_-rho*syT);
	  H_ = G*H_*G.transpose () + rho*s*s.transpose ();
	  alpha_ = alphaMax - .8*(alphaMax - alpha_);
	  x0 = x1; grad0 = grad1;
	  ++iter;
	}
	PathVectorPtr_t result = PathVector::create (configSize_, numberDofs_);
	vectorToPath (x1, result);
	return result;
      }

      GradientBased::GradientBased (const Problem& problem) :
	PathOptimizer (problem), cost_ (), robot_ (problem.robot ()),
    configSize_ (robot_->configSize ()), numberDofs_ (robot_->numberDof ())
      {
	distance_ = HPP_DYNAMIC_PTR_CAST (WeighedDistance, problem.distance ());
	if (!distance_) {
	  throw std::runtime_error
	    ("Distance is not of type WeighedDistance.");
	}
	steeringMethod_ = SteeringMethodStraight::create (robot_, distance_);
      }

      void GradientBased::pathToVector (const PathVectorPtr_t& path,
					vectorOut_t x1) const
      {
	size_type index = 0;
	for (std::size_t i=0; i < path->numberPaths () - 1; ++i) {
	  const PathPtr_t& localPath = path->pathAtRank (i);
	  value_type t1 = localPath->timeRange ().second;
	  (*localPath) (x1.segment (index, configSize_), t1);
	  index += configSize_;
	}
	assert (index == x1.size ());
      }

      void GradientBased::vectorToPath (vectorIn_t x,
					const PathVectorPtr_t& result) const
      {
	Configuration_t q0 = initial_, q1;
	size_type index = 0;
	while (index < x.size ()) {
	  q1 = x.segment (index, configSize_);
	  PathPtr_t p = (*steeringMethod_) (q0, q1);
	  result->appendPath (p);
	  q0 = q1;
	  index += configSize_;
	}
	q1 = end_;
	PathPtr_t p = (*steeringMethod_) (q0, q1);
	result->appendPath (p);
      }

      void GradientBased::integrate (vectorIn_t x0, vectorIn_t step,
				     vectorOut_t x1) const
      {
	size_type indexConfig = 0;
	size_type indexVelocity = 0;
	while (indexConfig < x0.size ()) {
	  hpp::model::integrate (robot_, x0.segment (indexConfig, configSize_),
				 step.segment (indexVelocity, numberDofs_),
				 x1.segment (indexConfig, configSize_));
	  indexConfig += configSize_;
	  indexVelocity += numberDofs_;
	}
	assert (indexVelocity == step.size ());
      }

    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
