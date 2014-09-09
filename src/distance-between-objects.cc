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

#include <fcl/distance.h>

#include <hpp/model/collision-object.hh>
#include <hpp/model/body.hh>
#include <hpp/model/device.hh>
#include <hpp/model/distance-result.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/distance-between-objects.hh>

namespace hpp {
  namespace core {
    void DistanceBetweenObjects::addObstacle
    (const CollisionObjectPtr_t& object)
    {
      using model::DISTANCE;
      const JointVector_t& jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
	   ++it) {
	JointPtr_t joint = *it;
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
	  const ObjectVector_t& bodyObjects = body->innerObjects (DISTANCE);
	  for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	       itInner != bodyObjects.end (); ++itInner) {
	    collisionPairs_.push_back (CollisionPair_t (*itInner, object));
	    distanceResults_.push_back (DistanceResult ());
	  }
	}
      }
    }

    void DistanceBetweenObjects::obstacles (const ObjectVector_t& obstacles)
    {
      for (ObjectVector_t::const_iterator itObj = obstacles.begin ();
	   itObj != obstacles.end (); ++itObj) {
	addObstacle (*itObj);
      }
    }

    void DistanceBetweenObjects::computeDistances ()
    {
      DistanceResults_t::size_type offset = 0;
      fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
	distanceResults_ [offset].fcl.clear ();
	const CollisionObjectPtr_t& obj1 = itCol->first;
	const CollisionObjectPtr_t& obj2 = itCol->second;
	fcl::distance (obj1->fcl ().get (), obj2->fcl ().get (),
		       distanceRequest, distanceResults_ [offset].fcl);
	distanceResults_ [offset].innerObject = obj1;
	distanceResults_ [offset].outerObject = obj2;
	++offset;
      }
    }

    DistanceBetweenObjects::DistanceBetweenObjects  (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
