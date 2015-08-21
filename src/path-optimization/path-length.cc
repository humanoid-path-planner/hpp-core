//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Mylene Campana
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

#include <Eigen/LU>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/path-optimization/path-length.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      PathLengthPtr_t PathLength::create
      (const WeighedDistancePtr_t& distance, const PathVectorPtr_t& path)
      {
	PathLength* ptr = new PathLength (distance, path);
	return PathLengthPtr_t (ptr);
      }

      PathLength::PathLength (const WeighedDistancePtr_t& distance,
			      const PathVectorPtr_t& path) :
	Cost ((path->numberPaths () - 1) * path->outputSize (),
	      (path->numberPaths () - 1) * path->outputDerivativeSize (),
	      "path length"),
	nbPaths_ (path->numberPaths ()), distance_ (distance),
	robot_ (distance->robot ()), configSize_ (path->outputSize ()),
	numberDofs_ (path->outputDerivativeSize ())
      {
	assert (nbPaths_ >= 1);
	assert (configSize_ == robot_->configSize ());
	assert (numberDofs_ == robot_->numberDof ());
	// Check that path is a list of straight interpolations
	for (std::size_t i=0; i<nbPaths_; ++i) {
	  if (!HPP_DYNAMIC_PTR_CAST (const StraightPath,
				     path->pathAtRank (i))) {
	    throw std::runtime_error
	      ("Path is not composed of StraighPath instances.");
	  }
	}
	// Store first and last way points.
	PathPtr_t first = path->pathAtRank (0);
	initial_ = (*first) (path->timeRange ().first);
	PathPtr_t last = path->pathAtRank (nbPaths_-1);
	end_ = (*last) (last->timeRange ().second);
	computeLambda (path);
      }

      void PathLength::computeLambda (const PathVectorPtr_t& path) const
      {
	lambda_.resize (nbPaths_);
	lambda_.setZero ();
	value_type lambdaMax = 0;
	for (std::size_t i=0; i < nbPaths_; ++i) {
	  value_type d = (*distance_) (path->pathAtRank (i)->initial (),
				       path->pathAtRank (i)->end ());
	  lambda_ [i] = d;
	  if (d > lambdaMax) lambdaMax = d;
	}
	for (std::size_t i=0; i < nbPaths_; ++i) {
	  value_type d (lambda_ [i]);
	  if (d > 1e-6*lambdaMax)
	    lambda_ [i] = 1./d;
	  else
	    lambda_ [i] = 1e6/lambdaMax;
	}

	hppDout (info, "computed lambda_ = " << lambda_.transpose ());
      }

      void PathLength::impl_compute (vectorOut_t result,
				     vectorIn_t argument) const
      {
	size_type index = 0;
	value_type d = (*distance_)
	  (initial_, argument.segment (index, configSize_));
	value_type cost = d*d * lambda_ [0];
	for (std::size_t i=0; i < nbPaths_ - 2; ++i) {
	  value_type d = (*distance_) (argument.segment (index, configSize_),
				       argument.segment (index + configSize_,
							 configSize_));
	  cost += d*d * lambda_ [i+1];
	  index += configSize_;
	}
	d = (*distance_) (argument.segment (index, configSize_), end_);
	cost += d*d * lambda_ [nbPaths_ - 1];
	result [0] = .5*cost;
      }

      void PathLength::impl_jacobian (matrixOut_t jacobian,
				      vectorIn_t argument) const
      {
	size_type indexConfig = 0;
	size_type indexVelocity = 0;
	vector_t u1 (numberDofs_);

	// Prepare first waypoint gradient computation
	hpp::model::difference (robot_,
				argument.segment (indexConfig, configSize_),
				initial_, u1);
	std::size_t rank = 0;
	for (JointVector_t::const_iterator itJoint =
	       robot_->getJointVector ().begin (); itJoint !=
	       robot_->getJointVector ().end (); ++itJoint) {
	  if ((*itJoint)->numberDof () != 0) {
	    value_type w = distance_->getWeight (rank);
	    u1.segment ((*itJoint)->rankInVelocity (),
			(*itJoint)->numberDof ()) *= w*w;
	    ++rank;
	  }
	}
	vector_t u2 (numberDofs_);

	// Prepare and compute first and other waypoints gradients
	for (std::size_t i=0; i < nbPaths_ - 2; ++i) {
	  hpp::model::difference (robot_,
				  argument.segment (indexConfig + configSize_,
						    configSize_),
				  argument.segment (indexConfig, configSize_),
				  u2);

	  rank = 0;
	  for (JointVector_t::const_iterator itJoint =
		 robot_->getJointVector ().begin (); itJoint !=
		 robot_->getJointVector ().end (); ++itJoint) {
	    if ((*itJoint)->numberDof () != 0) {
	      value_type w = distance_->getWeight (rank);
	      u2.segment ((*itJoint)->rankInVelocity (),
			  (*itJoint)->numberDof ()) *= w*w;
	      ++rank;
	    }
	  }
	  jacobian.block (0, indexVelocity, 1, numberDofs_) =
	    (u1* lambda_ [i] - u2* lambda_ [i+1]).transpose ();
	  indexConfig += configSize_;
	  indexVelocity += numberDofs_;
	  u1 = u2;
	}//forStraightPaths

	// Prepare last waypoint gradient computation
	hpp::model::difference (robot_, end_,
				argument.segment (indexConfig, configSize_),
				u2);
	rank = 0;
	for (JointVector_t::const_iterator itJoint =
	       robot_->getJointVector ().begin (); itJoint !=
	       robot_->getJointVector ().end (); ++itJoint) {
	  if ((*itJoint)->numberDof () != 0) {
	    value_type w = distance_->getWeight (rank);
	    u2.segment ((*itJoint)->rankInVelocity (),
			(*itJoint)->numberDof ()) *= w*w;
	    ++rank;
	  }
	}
	// Compute last waypoint gradient
	jacobian.block (0, indexVelocity, 1, numberDofs_) =
	  (u1 * lambda_ [nbPaths_-2] -
	   u2 * lambda_ [nbPaths_-1]).transpose ();
	indexConfig += configSize_;
	indexVelocity += numberDofs_;
	assert (indexConfig == argument.size ());
	assert (indexVelocity == jacobian.cols ());
      }

      void PathLength::hessian (matrixOut_t result) const
      {
	assert (result.rows () == inputDerivativeSize ());
	assert (result.cols () == inputDerivativeSize ());

	size_type n = nbPaths_ - 1;
	// Fill inverse weight matrix
	matrix_t W2 (numberDofs_, numberDofs_); W2.setZero ();
	size_type index = 0;
	size_type rank = 0;
	const JointVector_t & jv (robot_->getJointVector ());
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); ++itJoint) {
	  size_type s = (*itJoint)->numberDof ();
	  if (s > 0) {
	    value_type w = distance_->getWeight (rank);
	    hppDout (info, "rank = " << rank << ", joint = " <<
		     (*itJoint)->name () << ", weight = " << w);
	    while (s>0) {
	      W2 (index, index) = w*w;
	      --s;
	      ++index;
	    }
	    ++rank;
	  }
	}
	assert (index == numberDofs_);

	matrix_t hessian (inputDerivativeSize (), inputDerivativeSize ());
	hessian.setZero ();
	for (size_type i = 0; i < n-1; ++i) {
	  hessian.block (i*numberDofs_, i*numberDofs_,
			 numberDofs_, numberDofs_) =
	    (lambda_ [i] +  lambda_ [i+1]) * W2;
	  hessian.block ((i+1)*numberDofs_, i*numberDofs_,
			 numberDofs_, numberDofs_) = -lambda_ [i+1] * W2;
	  hessian.block (i*numberDofs_, (i+1)*numberDofs_,
			 numberDofs_, numberDofs_) = -lambda_ [i+1] * W2;
	}
	hessian.block ((n-1)*numberDofs_, (n-1)*numberDofs_, numberDofs_,
		       numberDofs_) = (lambda_ [n-1] + lambda_[n]) * W2;
	result = hessian;
      }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
