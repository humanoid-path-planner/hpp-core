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

#ifndef HPP_CORE_DISCRETIZED_COLLISION_CHECKING
# define HPP_CORE_DISCRETIZED_COLLISION_CHECKING

# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    /// Validation of path by collision checking at discretized parameter values
    ///
    /// Should be replaced soon by a better algorithm
    class HPP_CORE_DLLAPI DiscretizedCollisionChecking : public PathValidation
    {
    public:
      static DiscretizedCollisionCheckingPtr_t
      create (const DevicePtr_t& robot, const value_type& stepSize);
      virtual bool validate (const PathPtr_t& path, bool reverse,
			     PathPtr_t& validPart);
      /// Add an obstacle
      /// \param object obstacle added
      virtual void addObstacle (const CollisionObjectPtr_t&);
    protected:
      DiscretizedCollisionChecking (const DevicePtr_t& robot,
				    const value_type& stepSize);
    private:
      DevicePtr_t robot_;
      CollisionValidationPtr_t collisionValidation_;
      value_type stepSize_;
    }; // class DiscretizedCollisionChecking
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_DISCRETIZED_COLLISION_CHECKING
