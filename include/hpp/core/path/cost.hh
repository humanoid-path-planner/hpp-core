// Copyright (c) 2025 Centre National de la Recherche Scientifique
// Authors: Florent Lamiraux (florent.lamiraux@laas.fr)
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

#ifndef HPP_CORE_PATH_COST_HH
#define HPP_CORE_PATH_COST_HH

#include <hpp/core/path/cost.hh>

namespace hpp {
namespace core {
namespace path {
/// Cost of a path
///
/// Useful for searching optimal path in a roadmap
class HPP_CORE_DLLAPI Cost {
public:
  /// Evaluate the cost of a path
  virtual value_type eval(const PathConstPtr_t& path) = 0;
}; //class Cost

namespace cost{
/// Length of a path as cost
  class HPP_CORE_DLLAPI Length : public Cost {
public:
  static LengthPtr_t create() {
    return LengthPtr_t(new Length());
  }
  virtual value_type eval(const PathConstPtr_t& path) {
    return path->length();
  }
}; // class Length
} // namespace cost
} // namespace path
} // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_COST_HH
