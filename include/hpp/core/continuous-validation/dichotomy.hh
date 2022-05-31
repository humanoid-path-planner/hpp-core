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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH
#define HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH

#include <hpp/core/continuous-validation.hh>

namespace hpp {
namespace core {
namespace continuousValidation {
/// \addtogroup validation
/// \{

/// Continuous validation of a path
///
/// This class is a specialization of ContinuousValidation.
///
/// The interval validation is performed by
/// \li validating the beginning of the interval (or the end if paramater
///     reverse is set to true when calling
///     ContinuousValidation::validate),
/// \li validate intervals centered at the middle of the biggest non
///     tested interval.
/// See <a href="continuous-validation.pdf"> this document </a>
/// for details.
class HPP_CORE_DLLAPI Dichotomy : public ContinuousValidation {
 public:
  /// Create instance and return shared pointer
  /// \param robot the robot for which continuous validation is performed,
  /// \param tolerance maximal penetration allowed.
  static DichotomyPtr_t create(const DevicePtr_t& robot,
                               const value_type& tolerance);

  virtual ~Dichotomy();

 protected:
  /// Constructor
  /// \param robot the robot for which continuous validation is performed,
  /// \param tolerance maximal penetration allowed.
  Dichotomy(const DevicePtr_t& robot, const value_type& tolerance);
  /// Store weak pointer to itself
  void init(const DichotomyWkPtr_t weak);

 private:
  // Weak pointer to itself
  DichotomyWkPtr_t weak_;
  bool validateStraightPath(IntervalValidations_t& bodyPairCollisions,
                            const PathPtr_t& path, bool reverse,
                            PathPtr_t& validPart,
                            PathValidationReportPtr_t& report);
  template <bool reverse>
  bool validateStraightPath(IntervalValidations_t& bodyPairCollisions,
                            const PathPtr_t& path, PathPtr_t& validPart,
                            PathValidationReportPtr_t& report);
};  // class Dichotomy
}  // namespace continuousValidation
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH
