// Copyright (c) 2019 CNRS
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

#include <hpp/core/config-projector.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/straight-path.hh>

namespace hpp {
namespace core {
namespace steeringMethod {
PathPtr_t Straight::impl_compute(ConfigurationIn_t q1,
                                 ConfigurationIn_t q2) const {
  value_type length = (*problem()->distance())(q1, q2);
  ConstraintSetPtr_t c;
  if (constraints() && constraints()->configProjector()) {
    c = HPP_STATIC_PTR_CAST(ConstraintSet, constraints()->copy());
    c->configProjector()->rightHandSideFromConfig(q1);
    c->configProjector()->lineSearchType(ConfigProjector::Backtracking);
  } else {
    c = constraints();
  }
  PathPtr_t path = StraightPath::create(problem()->robot(), q1, q2, length, c);
  return path;
}
}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
