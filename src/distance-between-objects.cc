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

#include <hpp/core/distance-between-objects.hh>

#include <hpp/fcl/distance.h>

#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

#include <hpp/core/collision-pair.hh>

namespace hpp {
  namespace core {
    void DistanceBetweenObjects::addObstacle
    (const CollisionObjectConstPtr_t &object)
    {
      using pinocchio::DISTANCE;
      for (size_type i = 0; i < robot_->nbJoints(); ++i) {
        JointConstPtr_t joint = robot_->jointAt(i);
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
          for (size_type j = 0; j < body->nbInnerObjects(); ++j) {
	    collisionPairs_.emplace_back(body->innerObjectAt(j), object);
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
      distanceResults_.resize (collisionPairs_.size());
      std::size_t rank = 0;
      fcl::DistanceRequest distanceRequest (true, 0, 0);
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
    const CollisionObjectConstPtr_t& obj1 = itCol->first;
    const CollisionObjectConstPtr_t& obj2 = itCol->second;
    distanceResults_[rank].clear ();
    fcl::distance (obj1->geometry().get(), obj1->getFclTransform(),
               obj2->geometry().get(), obj2->getFclTransform(),
               distanceRequest, distanceResults_[rank]);
    ++rank;
      }
    }

    DistanceBetweenObjects::DistanceBetweenObjects  (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
