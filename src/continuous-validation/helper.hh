//
// Copyright (c) 2018 CNRS
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

#include <hpp/core/fwd.hh>
#include <iterator>

namespace hpp {
namespace core {
namespace continuousValidation {
inline const value_type& first(const interval_t& I, bool reverse) {
  return reverse ? I.second : I.first;
}
inline const value_type& second(const interval_t& I, bool reverse) {
  return first(I, !reverse);
}

inline const interval_t& begin(const std::list<interval_t>& I, bool reverse) {
  return reverse ? I.back() : I.front();
}
inline const interval_t& end(const std::list<interval_t>& I, bool reverse) {
  return begin(I, !reverse);
}
inline const interval_t& Nth(const std::list<interval_t>& I, int N,
                             bool reverse) {
  if (reverse) {
    std::list<interval_t>::const_reverse_iterator it(I.rbegin());
    std::advance(it, N);
    return *it;
  } else {
    std::list<interval_t>::const_iterator it(I.begin());
    std::advance(it, N);
    return *it;
  }
}
}  // namespace continuousValidation
}  // namespace core
}  // namespace hpp
