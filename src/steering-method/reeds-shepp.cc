// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/core/distance.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/joint/fwd.hpp>

namespace hpp {
namespace core {
namespace steeringMethod {
PathPtr_t ReedsShepp::impl_compute(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const {
  // TODO this should not be done here.
  // See todo in class ConstantCurvature
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
  value_type extraL = (*weighedDistance_)(q1, qEnd);

  value_type distance;
  PathVectorPtr_t path(
      reedsSheppPathOrDistance(device_.lock(), q1, q2, extraL, rho_, xyId_,
                               rzId_, wheels_, constraints(), false, distance));
  return path;
}

ReedsShepp::ReedsShepp(const ProblemConstPtr_t& problem)
    : CarLike(problem),
      weighedDistance_(WeighedDistance::create(problem->robot())),
      weak_() {}

ReedsShepp::ReedsShepp(const ProblemConstPtr_t& problem,
                       const value_type turningRadius, JointPtr_t xyJoint,
                       JointPtr_t rzJoint, std::vector<JointPtr_t> wheels)
    : CarLike(problem, turningRadius, xyJoint, rzJoint, wheels),
      weighedDistance_(WeighedDistance::create(problem->robot())) {}

/// Copy constructor
ReedsShepp::ReedsShepp(const ReedsShepp& other)
    : CarLike(other), weighedDistance_(other.weighedDistance_) {}

}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
