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

#ifndef HPP_CORE_CONFIG_VALIDATIONS_HH
#define HPP_CORE_CONFIG_VALIDATIONS_HH

#include <hpp/core/config-validation.hh>
#include <hpp/core/obstacle-user.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Validate a configuration with respect to collision
///
class HPP_CORE_DLLAPI ConfigValidations
    : public ConfigValidation,
      public ObstacleUserVector<ConfigValidationPtr_t> {
 public:
  static ConfigValidationsPtr_t create();

  /// Compute whether the configuration is valid
  ///
  /// \param config the config to check for validity,
  /// \retval validationReport report on validation. If non valid,
  ///         a validation report will be allocated and returned via this
  ///         shared pointer.
  /// \return whether the whole config is valid.
  virtual bool validate(const Configuration_t& config,
                        ValidationReportPtr_t& validationReport);
  /// Add a configuration validation object
  void add(const ConfigValidationPtr_t& configValidation);

  /// Return the number of config validations
  size_type numberConfigValidations() const;

 protected:
  ConfigValidations() = default;
  ConfigValidations(std::initializer_list<ConfigValidationPtr_t> validations)
      : ObstacleUserVector(validations) {};
};  // class ConfigValidation
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_CONFIG_VALIDATIONS_HH
