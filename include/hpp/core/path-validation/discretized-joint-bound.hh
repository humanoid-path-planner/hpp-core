//
// Copyright (c) 2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_CORE_PATH_VALIDATION_DISCRETIZED_JOINT_BOUND_HH
# define HPP_CORE_PATH_VALIDATION_DISCRETIZED_JOINT_BOUND_HH

#include <hpp/core/path-validation/discretized.hh>

namespace hpp {
  namespace core {
    namespace pathValidation {
      /// \addtogroup validation
      /// \{

      /// Validation of path by checking joint bounds at discretized parameter values
      DiscretizedPtr_t createDiscretizedJointBound (
          const DevicePtr_t& robot, const value_type& stepSize);

        /// \}
    } // namespace pathValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_VALIDATION_DISCRETIZED_JOINT_BOUND_HH
