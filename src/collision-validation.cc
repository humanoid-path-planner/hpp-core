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

#include <fcl/collision.h>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/collision-validation.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    typedef model::JointConfiguration* JointConfigurationPtr_t;
    CollisionValidationPtr_t CollisionValidation::create
    (const DevicePtr_t& robot)
    {
      CollisionValidation* ptr = new CollisionValidation (robot);
      return CollisionValidationPtr_t (ptr);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					bool report)
    {
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics ();
      bool collision = robot_->collisionTest ();
      if (collision) return false;
      fcl::CollisionRequest collisionRequest (1, false, false, 1, false, true,
					      fcl::GST_INDEP);
      fcl::CollisionResult collisionResult;
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
	if (fcl::collide (itCol->first->fcl ().get (),
			  itCol->second->fcl ().get (),
			  collisionRequest, collisionResult) != 0) {
	  collision = true;
	  break;
	}
      }
      if (collision && report) {
	std::ostringstream oss ("Configuration in collision: ");
	oss << displayConfig (config);
	throw std::runtime_error (oss.str ());
      }
      return !collision;
    }

    void CollisionValidation::addObstacle (const CollisionObjectPtr_t& object)
    {
      using model::COLLISION;
      const JointVector_t& jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
	   ++it) {
	JointPtr_t joint = *it;
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
	  const ObjectVector_t& bodyObjects = body->innerObjects (COLLISION);
	  for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	       itInner != bodyObjects.end (); ++itInner) {
	    collisionPairs_.push_back (CollisionPair_t (*itInner, object));
	  }
	}
      }
    }

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
