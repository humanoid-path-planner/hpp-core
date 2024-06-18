//
// Copyright (c) 2017 CNRS
// Authors: Diane Bury
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

#ifndef HPP_CORE_PATH_VALIDATIONS_HH
#define HPP_CORE_PATH_VALIDATIONS_HH

#include <hpp/core/obstacle-user.hh>
#include <hpp/core/path-validation.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Validation of a path with multiple path validation methods
///
/// Apply several path validation methods to the path parameter
class HPP_CORE_DLLAPI PathValidations
    : public PathValidation,
      public ObstacleUserVector<PathValidationPtr_t> {
 public:
  static PathValidationsPtr_t create();

  /// Compute the largest valid interval starting from the path beginning
  ///
  /// \param path the path to check for validity,
  /// \param reverse if true check from the end,
  /// \retval the extracted valid part of the path, pointer to path if
  ///         path is valid.
  /// \retval report information about the validation process. A report
  ///         is allocated if the path is not valid.
  /// \return whether the whole path is valid.
  virtual bool validate(const PathPtr_t& path, bool reverse,
                        PathPtr_t& validPart,
                        PathValidationReportPtr_t& report);

  /// Validate a single configuration
  /// \param q input configuration,
  /// \retval report validation report.
  /// The default implementation builds a straight path of length 0
  /// with the input configuration and validates the path.
  virtual bool validate(ConfigurationIn_t q, ValidationReportPtr_t& report);

  /// Add a path validation object
  virtual void addPathValidation(const PathValidationPtr_t& pathValidation);

  virtual ~PathValidations() {};

 protected:
  PathValidations();
};  // class PathValidations
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_VALIDATIONS_HH
