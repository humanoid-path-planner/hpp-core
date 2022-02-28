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

#ifndef HPP_CORE_JOINT_BOUND_VALIDATION_HH
# define HPP_CORE_JOINT_BOUND_VALIDATION_HH

# include <hpp/pinocchio/joint.hh>
# include <hpp/core/config-validation.hh>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// report returned when a configuration is not within the bounds
    class HPP_CORE_DLLAPI JointBoundValidationReport : public ValidationReport
    {
    public:
      JointBoundValidationReport (const JointConstPtr_t& joint, size_type rank,
				  value_type lowerBound, value_type upperBound,
				  value_type value) :
	ValidationReport (),
	joint_ (joint), rank_ (rank), lowerBound_ (lowerBound),
	upperBound_ (upperBound), value_ (value) 
	{
	}
      /// Print report in a stream
      virtual std::ostream& print (std::ostream& os) const
      {
        if (joint_) {
          os << "Joint " << joint_->name () << ", rank: " << rank_
            << ", value out of range: " << value_ << " not in ["
            << lowerBound_ << ", " << upperBound_ << "]";
        } else {
          os << "Extra config space at rank: " << rank_
             << ", value out of range: " << value_ << " not in ["
             << lowerBound_ << ", " << upperBound_ << "]";
        }
	return os;
      }

      /// Joint the configuration value is out of bounds
      JointConstPtr_t joint_;
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
      /// \retval validationReport report on validation. If non valid,
      ///         a validation report will be allocated and returned via this
      ///         shared pointer.
      /// \return whether the whole config is valid.
      bool validate (const Configuration_t& config,
		     ValidationReportPtr_t& validationReport);
    protected:
      JointBoundValidation (const DevicePtr_t& robot);
    private:
      DevicePtr_t robot_;
    }; // class ConfigValidation
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_JOINT_BOUND_VALIDATION_HH
