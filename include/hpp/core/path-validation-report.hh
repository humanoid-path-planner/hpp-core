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

#ifndef HPP_CORE_PATH_VALIDATION_REPORT_HH
#define HPP_CORE_PATH_VALIDATION_REPORT_HH

#include <hpp/core/validation-report.hh>
#include <hpp/util/indent.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Abstraction of path validation report
///
/// This class is aimed at being derived to store information relative to
/// various PathValidation derived classes.
struct HPP_CORE_DLLAPI PathValidationReport : public ValidationReport {
  PathValidationReport() : ValidationReport(), configurationReport() {}
  PathValidationReport(const value_type& param,
                       const ValidationReportPtr_t& report)
      : parameter(param), configurationReport(report) {}

  virtual ~PathValidationReport() {};

  virtual std::ostream& print(std::ostream& os) const {
    os << "Invalid configuration at parameter " << parameter << iendl
       << incindent;
    if (!configurationReport)
      os << "No ValidationReport";
    else
      os << *configurationReport;
    return os << decindent;
  }

  value_type getParameter() { return parameter; }

  void setParameter(value_type p) { parameter = p; }

  /// Parameter of the path where a invalid configuration has been found
  value_type parameter;
  ValidationReportPtr_t configurationReport;
};  // class PathValidationReport
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_VALIDATION_REPORT_HH
