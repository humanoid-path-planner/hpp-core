//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
