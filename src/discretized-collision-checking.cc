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

#include <hpp/model/device.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/core/discretized-collision-checking.hh>

namespace hpp {
  namespace core {

    DiscretizedCollisionCheckingPtr_t
    DiscretizedCollisionChecking::create (const DevicePtr_t& robot,
					  const value_type& stepSize)
    {
      DiscretizedCollisionChecking* ptr =
	new DiscretizedCollisionChecking(robot, stepSize);
      return DiscretizedCollisionCheckingPtr_t (ptr);
    }

    void DiscretizedCollisionChecking::addObstacle
    (const CollisionObjectPtr_t& object)
    {
      collisionValidation_->addObstacle (object);
    }

    bool DiscretizedCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      bool valid = true;
      if (reverse) {
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmax;
	value_type t = tmax - stepSize_;
	unsigned finished = 0;
	while (finished < 2 && valid) {
	  Configuration_t q = (*path) (t);
	  if (!collisionValidation_->validate (q)) {
	    valid = false;
	  } else {
	    lastValidTime = t;
	    t -= stepSize_;
	  }
	  if (t < tmin) {
	    t = tmin;
	    finished++;
	  }
	}
	if (valid) {
	  validPart = path;
	  return true;
	} else {
	  validPart = path->extract (std::make_pair (lastValidTime, tmax));
	  return false;
	}
      } else {
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmin;
	value_type t = tmin + stepSize_;
	unsigned finished = 0;
	while (finished < 2 && valid) {
	  Configuration_t q = (*path) (t);
	  if (!collisionValidation_->validate (q)) {
	    valid = false;
	  } else {
	    lastValidTime = t;
	    t += stepSize_;
	  }
	  if (t > tmax) {
	    t = tmax;
	    finished ++;
	  }
	}
	if (valid) {
	  validPart = path;
	  return true;
	} else {
	  validPart = path->extract (std::make_pair (tmin, lastValidTime));
	  return false;
	}
      }
    }

    DiscretizedCollisionChecking::DiscretizedCollisionChecking
    (const DevicePtr_t& robot, const value_type& stepSize) :
      PathValidation (), robot_ (robot),
      collisionValidation_ (CollisionValidation::create (robot)),
      stepSize_ (stepSize)
    {
    }

  } // namespace core
} // namespace hpp
