//
// Copyright (c) 2022 CNRS Airbus S.A.S.
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

#include <hpp/core/path-validation.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/pinocchio/liegroup-space.hh>

namespace hpp {
namespace core {
bool PathValidation::validate(ConfigurationIn_t q,
                              ValidationReportPtr_t& report) {
  // Create a straight path of length 0 with the configuration.
  // The output space does not matter here since no Liegroup operation
  // will be performed. Thus, we use Rn.
  StraightPathPtr_t p(
      StraightPath::create(LiegroupSpace::Rn(q.size()), q, q,
                           std::make_pair<value_type, value_type>(0, 0)));
  PathPtr_t unused;
  PathValidationReportPtr_t r;
  bool res(this->validate(p, false, unused, r));
  if (r) report = r->configurationReport;
  return res;
}
}  // namespace core
}  // namespace hpp
