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

#ifndef HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH
# define HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH

# include <hpp/core/validation-report.hh>
# include <hpp/core/collision-validation-report.hh>

namespace hpp {
  namespace core {
    /// Abstraction of path validation report
    ///
    /// This class is aimed at being derived to store information relative to
    /// various PathValidation derived classes.
    struct HPP_CORE_DLLAPI CollisionPathValidationReport :
      public ValidationReport
    {
      /// Parameter of the path where a collision has been found
      value_type collisionParameter;
      CollisionValidationReport collision;
    }; // class PathValidationReport
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH
