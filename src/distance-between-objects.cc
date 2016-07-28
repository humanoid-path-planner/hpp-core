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

#include <hpp/fcl/distance.h>

#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/core/distance-between-objects.hh>

namespace hpp {
  namespace core {
    void DistanceBetweenObjects::addObstacle
    (const CollisionObjectConstPtr_t &object)
    {
      using pinocchio::DISTANCE;
      const JointVector_t& jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
	   ++it) {
    JointConstPtr_t joint = *it;
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
      const ObjectVector_t& bodyObjects = body->innerObjects ();
	  for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	       itInner != bodyObjects.end (); ++itInner) {
	    collisionPairs_.push_back (CollisionPair_t (*itInner, object));
	  }
	}
      }
    }

    void DistanceBetweenObjects::obstacles (const ObjectStdVector_t& obstacles)
    {
      for (ObjectStdVector_t::const_iterator itObj = obstacles.begin ();
	   itObj != obstacles.end (); ++itObj) {
	addObstacle (*itObj);
      }
    }

    void DistanceBetweenObjects::computeDistances ()
    {
      fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
    const CollisionObjectConstPtr_t& obj1 = itCol->first;
    const CollisionObjectConstPtr_t& obj2 = itCol->second;
    fcl::DistanceResult fclDistance;
    fcl::distance (obj1->fcl (), obj2->fcl (),
               distanceRequest, fclDistance);
    distanceResults_.push_back(DistanceResult(fclDistance,obj1->indexInModel(),obj2->indexInModel()));
      }
    }

    DistanceBetweenObjects::DistanceBetweenObjects  (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
