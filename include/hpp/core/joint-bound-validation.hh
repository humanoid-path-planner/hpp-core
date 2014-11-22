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

#ifndef HPP_CORE_JOINT_BOUND_VALIDATION_HH
# define HPP_CORE_JOINT_BOUND_VALIDATION_HH

# include <hpp/core/config-validation.hh>

namespace hpp {
  namespace core {
    /// Exception thrown when a configuration is not within the bounds
    class HPP_CORE_DLLAPI JointBoundException : public std::runtime_error
    {
    public:
      JointBoundException (const std::string& what, const JointPtr_t& joint,
			   size_type rank, value_type lowerBound,
			   value_type upperBound, value_type value) :
	runtime_error (what),
	joint_ (joint), rank_ (rank), lowerBound_ (lowerBound),
	upperBound_ (upperBound), value_ (value) 
	{
	}
      /// Joint the configuration value is out of bounds
      JointPtr_t joint_;
      /// degree of freedom in the joint (usually 0)
      size_type rank_;
      /// lower bound
      value_type lowerBound_;
      /// upper bound
      value_type upperBound_;
      /// configuration value
      value_type value_;
    };

    /// Validate a configuration with respect to joint bounds
    ///
    class HPP_CORE_DLLAPI JointBoundValidation : public ConfigValidation
    {
    public:
      static JointBoundValidationPtr_t create (const DevicePtr_t& robot);
      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \param throwIfInValid if true throw an exception if config is invalid.
      /// \return whether the whole config is valid.
      virtual bool validate (const Configuration_t& config,
			     bool throwIfInValid = false);

    protected:
      JointBoundValidation (const DevicePtr_t& robot);
    private:
      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport unused by this class,
      /// \param throwIfInValid if true throw an exception if config is invalid.
      /// \return whether the whole config is valid.
      virtual bool validate (const Configuration_t& config,
			     ValidationReport& validationReport,
			     bool throwIfInValid = false);
      DevicePtr_t robot_;
    }; // class ConfigValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_JOINT_BOUND_VALIDATION_HH
