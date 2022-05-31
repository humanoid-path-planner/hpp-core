// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_CORE_PATH_MATH_HH
#define HPP_CORE_PATH_MATH_HH

namespace hpp {
namespace core {
namespace path {
template <int N>
struct binomials {
  typedef Eigen::Matrix<size_type, N, 1> Factorials_t;

  static inline const Factorials_t& factorials() {
    static Factorials_t ret(Factorials_t::Zero());
    if (ret(0) == 0) {
      ret(0) = 1;
      for (size_type i = 1; i < N; ++i) ret(i) = ret(i - 1) * i;
    }
    return ret;
  }

  static inline size_type binomial(size_type n, size_type k) {
    const Factorials_t& factors = factorials();
    assert(n >= k && k >= 0);
    assert(n < N);
    size_type denom = factors(k) * factors(n - k);
    return factors(n) / denom;
  }
};
}  //   namespace path
}  //   namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_MATH_HH
