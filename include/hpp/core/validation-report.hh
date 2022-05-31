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

#ifndef HPP_CORE_VALIDATION_REPORT_HH
#define HPP_CORE_VALIDATION_REPORT_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Abstraction of validation report for paths and configurations
///
/// This class is aimed at being derived to store information relative to
/// various Validation derived classes
/// \li CollisionValidation,
/// \li collision related PathValidation classes.
class HPP_CORE_DLLAPI ValidationReport {
 public:
  virtual ~ValidationReport() {}
  /// Write report in a stream
  virtual std::ostream& print(std::ostream& os) const = 0;
};  // class ValidationReport
inline std::ostream& operator<<(std::ostream& os,
                                const ValidationReport& report) {
  return report.print(os);
}
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_VALIDATION_REPORT_HH
