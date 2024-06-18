// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_PROJECTION_ERROR_HH
#define HPP_CORE_PROJECTION_ERROR_HH

#include <exception>
#include <hpp/core/path-validation-report.hh>

namespace hpp {
namespace core {
struct HPP_CORE_DLLAPI projection_error : public std::exception {
  projection_error() : msg_() {}

  projection_error(const std::string& msg) : msg_(msg) {}

  projection_error(const projection_error& other)
      : std::exception(other), msg_(other.msg_) {}

  virtual ~projection_error() noexcept {};
  virtual const char* what() const noexcept { return msg_.c_str(); };

  std::string msg_;
};

/// \addtogroup validation
/// \{

/// Handles projection errors when evaluating a path.
struct HPP_CORE_DLLAPI ProjectionError : public ValidationReport {
  ProjectionError(const std::string& msg = "") : ValidationReport(), msg(msg) {}

  virtual ~ProjectionError() {};

  virtual std::ostream& print(std::ostream& os) const {
    os << "Projection error";
    if (!msg.empty()) os << " (" << msg << ')';
    return os;
  }

  std::string msg;
};  // class ProjectionError
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PROJECTION_ERROR_HH
