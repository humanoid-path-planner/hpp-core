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

#include <Eigen/LU>
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
      }

      void PathLength::impl_compute (vectorOut_t result,
				     vectorIn_t argument) const
      {
	size_type index = 0;
	value_type d = (*distance_)
	  (initial_, argument.segment (index, configSize_));
	value_type cost = d*d;
	for (std::size_t i=0; i < nbPaths_ - 2; ++i) {
	  value_type d = (*distance_) (argument.segment (index, configSize_),
				    argument.segment (index + configSize_,
						      configSize_));
	  cost += d*d;
	  index += configSize_;
	}
	d = (*distance_) (argument.segment (index, configSize_), end_);
	cost += d*d;

	result [0] = .5*cost;
      }

      void PathLength::impl_jacobian (matrixOut_t jacobian,
				      vectorIn_t argument) const
      {
	size_type indexConfig = 0;
	size_type indexVelocity = 0;
	// Velocity type
	vector_t u1 (numberDofs_);
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
	// Velocity type
	vector_t u2 (numberDofs_);

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
	    (u1 - u2).transpose ();
	  indexConfig += configSize_;
	  indexVelocity += numberDofs_;
	  u1 = u2;
	}
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
	jacobian.block (0, indexVelocity, 1, numberDofs_) =
	  (u1 - u2).transpose ();
	indexConfig += configSize_;
	indexVelocity += numberDofs_;
	assert (indexConfig == argument.size ());
	assert (indexVelocity == jacobian.cols ());
      }

      void PathLength::hessianInverse (matrixOut_t result) const
      {
	assert (result.rows () == inputDerivativeSize ());
	assert (result.cols () == inputDerivativeSize ());
	
	size_type n = nbPaths_;
	// Fill inverse weight matrix
	matrix_t inverseWeight (numberDofs_, numberDofs_);
	matrix_t weight (numberDofs_, numberDofs_);
	inverseWeight.setZero ();
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
	    w = 1./(w*w);
	    while (s>0) {
	      inverseWeight (index, index) = w;
	      --s;
	      ++index;
	    }
	    ++rank;
	  }
	}
	assert (index == numberDofs_);

	for (size_type j = 0; j < n-1; ++j) {
	  for (size_type k = 0; k <= j; ++k) {
	    result.block (j*numberDofs_, k*numberDofs_,
			  numberDofs_, numberDofs_) =
          (value_type) ((k+1)*(n-j-1))/((value_type)n)*inverseWeight;
	    if (k != j) {
	      result.block (k*numberDofs_, j*numberDofs_,
			    numberDofs_, numberDofs_) =
		result.block (j*numberDofs_, k*numberDofs_,
			      numberDofs_, numberDofs_);
	    }
	  }
	}
	// test product with Hessian
	weight = inverseWeight.inverse ();
	hppDout (info, "weight = " << weight);
	hppDout (info, "inverse weight = " << inverseWeight);
	matrix_t hessian (inputDerivativeSize (), inputDerivativeSize ());
	for (size_type i = 0; i < n-2; ++i) {
	  hessian.block (i*numberDofs_, i*numberDofs_,
			 numberDofs_, numberDofs_) = 2*weight;
	  hessian.block ((i+1)*numberDofs_, i*numberDofs_,
			 numberDofs_, numberDofs_) = -weight;
	  hessian.block (i*numberDofs_, (i+1)*numberDofs_,
			 numberDofs_, numberDofs_) = -weight;
	}
	hessian.block ((n-2)*numberDofs_, (n-2)*numberDofs_,
		       numberDofs_, numberDofs_) = 2*weight;
	hppDout (info, "Hessian = " << hessian);
	hppDout (info, "inverse Hessian = " << result);
	hppDout (info, "test: " <<
		 (hessian*result - matrix_t::Identity
		  (inputDerivativeSize (), inputDerivativeSize ())));
      }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
