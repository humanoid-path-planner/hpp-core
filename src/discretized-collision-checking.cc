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

#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/discretized-path-validation.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/joint-bound-validation.hh>

namespace hpp {
  namespace core {

    DiscretizedCollisionCheckingPtr_t
    DiscretizedCollisionChecking::create (const DevicePtr_t& robot,
					  const value_type& stepSize)
    {
      DiscretizedCollisionChecking* ptr (new DiscretizedCollisionChecking
					 (robot, stepSize));
      DiscretizedCollisionCheckingPtr_t shPtr (ptr);
      return shPtr;
    }

    DiscretizedCollisionChecking::DiscretizedCollisionChecking
    (const DevicePtr_t& robot, const value_type& stepSize) :
      DiscretizedPathValidation (robot, stepSize)
    {
      add (CollisionValidationPtr_t (CollisionValidation::create (robot)));
      add (JointBoundValidationPtr_t (JointBoundValidation::create (robot)));
    }

  } // namespace core
} // namespace hpp
