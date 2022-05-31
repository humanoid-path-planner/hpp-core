// Copyright (c) 2021 CNRS
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

#ifndef HPP_CORE_PATH_PLANNING_FAILED_HH
#define HPP_CORE_PATH_PLANNING_FAILED_HH

#include <exception>

namespace hpp {
namespace core {
struct HPP_CORE_DLLAPI path_planning_failed : public std::exception {
  path_planning_failed() : msg_() {}

  path_planning_failed(const std::string& msg) : msg_(msg) {}

  path_planning_failed(const path_planning_failed& other)
      : std::exception(other), msg_(other.msg_) {}

  virtual ~path_planning_failed() _GLIBCXX_USE_NOEXCEPT{};

  virtual const char* what() const noexcept { return msg_.c_str(); };

  std::string msg_;
};

}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_PLANNING_FAILED_HH
