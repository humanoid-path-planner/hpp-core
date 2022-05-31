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

#ifndef HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH
#define HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH

#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/path-validation-report.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Path validation report used for standard collision checking
struct HPP_CORE_DLLAPI CollisionPathValidationReport
    : public PathValidationReport {
  CollisionPathValidationReport() : PathValidationReport() {
    configurationReport =
        CollisionValidationReportPtr_t(new CollisionValidationReport);
  }

  CollisionPathValidationReport(const value_type& param,
                                const ValidationReportPtr_t& report)
      : PathValidationReport(param, report) {}
};  // struct CollisionPathValidationReport
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_COLLISION_PATH_VALIDATION_REPORT_HH
