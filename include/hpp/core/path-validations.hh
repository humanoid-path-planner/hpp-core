//
// Copyright (c) 2017 CNRS
// Authors: Diane Bury
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

#ifndef HPP_CORE_PATH_VALIDATIONS_HH
# define HPP_CORE_PATH_VALIDATIONS_HH

# include <hpp/core/obstacle-user.hh>
# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// Validation of a path with multiple path validation methods
    ///
    /// Apply several path validation methods to the path parameter
    class HPP_CORE_DLLAPI PathValidations :
      public PathValidation,
      public ObstacleUserVector<PathValidationPtr_t>
    {
    public:
      static PathValidationsPtr_t create ();

      /// Compute the largest valid interval starting from the path beginning
      ///
      /// \param path the path to check for validity,
      /// \param reverse if true check from the end,
      /// \retval the extracted valid part of the path, pointer to path if
      ///         path is valid.
      /// \retval report information about the validation process. A report
      ///         is allocated if the path is not valid.
      /// \return whether the whole path is valid.
      virtual bool validate (const PathPtr_t& path, bool reverse,
			     PathPtr_t& validPart,
			     PathValidationReportPtr_t& report);

      /// Add a path validation object
      virtual void addPathValidation (const PathValidationPtr_t& pathValidation);

      virtual ~PathValidations () {};
    protected:
      PathValidations ();
    }; // class PathValidations
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_VALIDATIONS_HH
