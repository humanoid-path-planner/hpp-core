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

#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/children-iterator.hh>
#include <hpp/core/weighed-distance.hh>
#include <Eigen/SVD>

namespace hpp {
  namespace core {
    std::ostream& operator<< (std::ostream& os, const std::vector <value_type>& v)
    {
      for (std::size_t i=0; i<v.size (); ++i) {
	os << v [i] << ",";
      }
      return os;
    }

    WeighedDistancePtr_t WeighedDistance::create (const DevicePtr_t& robot)
    {
      WeighedDistance* ptr = new WeighedDistance (robot);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t
    WeighedDistance::create (const DevicePtr_t& robot,
			     const std::vector <value_type>& weights)
    {
      WeighedDistance* ptr = new WeighedDistance (robot, weights);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t WeighedDistance::createCopy
	(const WeighedDistancePtr_t& distance)
    {
      WeighedDistance* ptr = new WeighedDistance (*distance);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    DistancePtr_t WeighedDistance::clone () const
    {
      return createCopy (weak_.lock ());
    }

    value_type WeighedDistance::getWeight( std::size_t rank ) const
    {
      return weights_[rank];
    }

    void WeighedDistance::setWeight (std::size_t rank, value_type weight )
    {
      if ( rank < weights_.size() ) 
      {
	weights_[rank] = weight;
      }
      else {
	std::ostringstream oss;
	oss << "Distance::setWeight : rank " << rank << " is out of range ("
	    << weights_.size () << ").";
	throw std::runtime_error(oss.str ());
      }
    } 

    WeighedDistance::WeighedDistance (const DevicePtr_t& robot) :
      robot_ (robot), weights_ ()
    {
      // Store computation flag
      Device_t::Computation_t flag = robot->computationFlag ();
      Device_t::Computation_t newflag = static_cast <Device_t::Computation_t>
	(flag | Device_t::JACOBIAN);
      robot->controlComputation (newflag);
      robot->computeForwardKinematics ();
      robot->controlComputation (flag);
      value_type minLength = std::numeric_limits <value_type>::infinity ();
      matrix_t jacobian; jacobian.resize (3, robot->numberDof ());
      const JointVector_t jointVector (robot->getJointVector ());
      for (JointVector_t::const_iterator it1 = jointVector.begin ();
	   it1 != jointVector.end (); ++it1) {
	if ((*it1)->numberDof () != 0) {
	  value_type length = 0;
	  std::size_t rank = (*it1)->rankInVelocity ();
	  std::size_t ncol = (*it1)->numberDof ();
	  matrix_t jointJacobian;
	  for (hpp::model::ChildrenIterator it2 (*it1); !it2.end (); ++it2){
	    //if ((*it2)->numberDof () != 0) { // allow anchors
	    // Get only three first lines of Jacobian
	    jointJacobian = (*it2)->jacobian ().block (0, rank, 3, ncol);
	    Eigen::JacobiSVD <matrix_t> svd (jointJacobian);
	    if (length < svd.singularValues () [0]) {
	      length = svd.singularValues () [0];
	    }
	    if (BodyPtr_t body = (*it2)->linkedBody ()) {
	      value_type radius = body->radius ();
	      jointJacobian = (*it2)->jacobian ().block (3, rank, 3, ncol);
	      Eigen::JacobiSVD <matrix_t> svd (jointJacobian);
	      if (length < radius*svd.singularValues () [0]) {
		length = radius*svd.singularValues () [0];
	      }
	    }
	  }
	  if (minLength > length && length > 0) minLength = length;
	  weights_.push_back (length);
	}
	for (std::size_t i=0; i < weights_.size (); ++i) {
	  if (weights_ [i] == 0) {
	    weights_ [i] = minLength;
	  }
	}
      }
    }

    WeighedDistance::WeighedDistance (const DevicePtr_t& robot,
				      const std::vector <value_type>& weights) :
      robot_ (robot), weights_ (weights)
    {
    }

    WeighedDistance::WeighedDistance (const WeighedDistance& distance) :
      robot_ (distance.robot_),
      weights_ (distance.weights_)
    {
    }

    void WeighedDistance::init (WeighedDistanceWkPtr_t self)
    {
      weak_ = self;
    }

    value_type WeighedDistance::impl_distance (ConfigurationIn_t q1,
					       ConfigurationIn_t q2) const
    {
      // Loop over robot joint and interpolate
      value_type res = 0;
      std::size_t i=0;
      const JointVector_t jointVector (robot_->getJointVector ());
      for (JointVector_t::const_iterator itJoint = jointVector.begin ();
	   itJoint != jointVector.end (); ++itJoint) {
	if ((*itJoint)->numberDof () != 0) {
	  value_type length = weights_ [i];
	  value_type distance =
	    (*itJoint)->configuration ()->distance
	    (q1, q2, (*itJoint)->rankInConfiguration ());
	  res += length * length * distance * distance;
	  ++i;
	}
      }
      return sqrt (res);
    }
  } //   namespace core
} // namespace hpp
