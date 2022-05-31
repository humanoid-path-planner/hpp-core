// Copyright (c) 2017, CNRS
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

#include <hpp/core/distance.hh>
#include <hpp/core/dubins-path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/dubins.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
namespace core {
namespace steeringMethod {
PathPtr_t Dubins::impl_compute(ConfigurationIn_t q1,
                               ConfigurationIn_t q2) const {
  Configuration_t qEnd(q2);
  qEnd.segment<2>(xyId_) = q1.segment<2>(xyId_);
  qEnd.segment<2>(rzId_) = q1.segment<2>(rzId_);
  // Do not take into account wheel joints in additional distance.
  for (std::vector<JointPtr_t>::const_iterator it = wheels_.begin();
       it != wheels_.end(); ++it) {
    size_type i = (*it)->rankInConfiguration();
    qEnd[i] = q1[i];
  }
  // The length corresponding to the non RS DoF
  DistancePtr_t d(problem()->distance());
  value_type extraL = (*d)(q1, qEnd);
  DubinsPathPtr_t path =
      DubinsPath::create(device_.lock(), q1, q2, extraL, rho_, xyId_, rzId_,
                         wheels_, constraints());
  return path;
}

Dubins::Dubins(const ProblemConstPtr_t& problem) : CarLike(problem), weak_() {}

Dubins::Dubins(const ProblemConstPtr_t& problem, const value_type turningRadius,
               JointPtr_t xyJoint, JointPtr_t rzJoint,
               std::vector<JointPtr_t> wheels)
    : CarLike(problem, turningRadius, xyJoint, rzJoint, wheels) {}

/// Copy constructor
Dubins::Dubins(const Dubins& other) : CarLike(other) {}

}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
