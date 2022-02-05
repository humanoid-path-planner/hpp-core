//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_PATH_VALIDATION_DISCRETIZED_HH
# define HPP_CORE_PATH_VALIDATION_DISCRETIZED_HH

# include <hpp/core/config-validations.hh>
# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    namespace pathValidation {
    /// \addtogroup validation
    /// \{

    /// Discretized validation of a path
    ///
    /// Apply some configuration validation algorithms at discretized values
    /// of the path parameter.
    class HPP_CORE_DLLAPI Discretized :
      public PathValidation,
      public ConfigValidations
    {
    public:
      static DiscretizedPtr_t create (const value_type& stepSize);
      static DiscretizedPtr_t create (const value_type& stepSize, 
          std::initializer_list<ConfigValidationPtr_t> validations);

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

      /// Validate a single configuration
      /// \param q input configuration,
      /// \retval report validation report.
      /// The default implementation builds a straight path of length 0
      /// with the input configuration and validates the path.
      virtual bool validate(ConfigurationIn_t q, ValidationReportPtr_t& report);

      virtual ~Discretized () {};
    protected:
      Discretized (const value_type& stepSize) : stepSize_ (stepSize) {}
      Discretized (const value_type& stepSize,
          std::initializer_list<ConfigValidationPtr_t> validations) :
        ConfigValidations (validations),
        stepSize_ (stepSize)
      {};

      value_type stepSize_;
    }; // class Discretized
    /// \}
    } // namespace pathValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_DISCRETIZED_PATH_VALIDATION_HH
