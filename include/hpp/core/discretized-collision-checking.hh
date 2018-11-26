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

# warning "This file is deprecated. Include <hpp/core/path-validation/discretized-collision-checking.hh> instead."
# include <hpp/core/path-validation/discretized-collision-checking.hh>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// \deprecated Use hpp::core::pathValidation::createDiscretizedCollisionChecking 
    class HPP_CORE_DLLAPI DiscretizedCollisionChecking
    {
    public:
      static pathValidation::DiscretizedPtr_t
      create (const DevicePtr_t& robot, const value_type& stepSize)
      {
        return pathValidation::createDiscretizedCollisionChecking (robot, stepSize);
      }
    }; // class DiscretizedCollisionChecking
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_DISCRETIZED_COLLISION_CHECKING
