//
// Copyright (c) 2016 CNRS
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

#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/joint/joint-generic.hpp>

namespace hpp {
namespace core {
namespace distance {

DistancePtr_t ReedsShepp::clone() const { return createCopy(weak_.lock()); }

ReedsSheppPtr_t ReedsShepp::create(const ProblemConstPtr_t& problem) {
  ReedsShepp* ptr(new ReedsShepp(problem));
  ReedsSheppPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}
ReedsSheppPtr_t ReedsShepp::create(const ProblemConstPtr_t& problem,
                                   const value_type& turningRadius,
                                   JointPtr_t xyJoint, JointPtr_t rzJoint,
                                   std::vector<JointPtr_t> wheels) {
  ReedsShepp* ptr(
      new ReedsShepp(problem, turningRadius, xyJoint, rzJoint, wheels));
  ReedsSheppPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ReedsSheppPtr_t ReedsShepp::createCopy(const ReedsSheppPtr_t& distance) {
  ReedsShepp* ptr(new ReedsShepp(*distance));
  ReedsSheppPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ReedsShepp::ReedsShepp(const ProblemConstPtr_t& problem)
    : Distance(),
      weighedDistance_(WeighedDistance::create(problem->robot())),
      device_(problem->robot()),
      xyId_(0),
      rzId_(2) {
  DevicePtr_t d(device_.lock());
  xy_ = d->getJointAtConfigRank(xyId_);
  rz_ = d->getJointAtConfigRank(rzId_);
  wheels_ = steeringMethod::getWheelsFromParameter(problem, rz_);
  turningRadius(problem->getParameter("SteeringMethod/Carlike/turningRadius")
                    .floatValue());
}

ReedsShepp::ReedsShepp(const ProblemConstPtr_t& problem,
                       const value_type& turningRadius, JointPtr_t xyJoint,
                       JointPtr_t rzJoint, std::vector<JointPtr_t> wheels)
    : Distance(),
      weighedDistance_(WeighedDistance::create(problem->robot())),
      device_(problem->robot()),
      rho_(turningRadius),
      xy_(xyJoint),
      rz_(rzJoint),
      xyId_(xy_->rankInConfiguration()),
      wheels_(wheels),
      weak_() {
  if (rz_->jointModel().shortname() == "JointModelPlanar") {
    rzId_ = rz_->rankInConfiguration() + 2;
  } else {
    rzId_ = rz_->rankInConfiguration();
  }
}

ReedsShepp::ReedsShepp(const ReedsShepp& other)
    : Distance(other),
      weighedDistance_(WeighedDistance::createCopy(other.weighedDistance_)),
      device_(other.device_),
      rho_(other.rho_),
      xy_(other.xy_),
      rz_(other.rz_),
      xyId_(other.xyId_),
      rzId_(other.rzId_),
      wheels_(other.wheels_),
      weak_() {}

void ReedsShepp::turningRadius(const value_type& rho) {
  if (rho <= 0)
    throw std::invalid_argument("Turning radius must be strictly positive.");
  rho_ = rho;
}

value_type ReedsShepp::impl_distance(ConfigurationIn_t q1,
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
  PathVectorPtr_t path(steeringMethod::reedsSheppPathOrDistance(
      device_.lock(), q1, q2, extraL, rho_, xyId_, rzId_, wheels_,
      ConstraintSetPtr_t(), true, distance));
  return distance;
}

void ReedsShepp::init(const ReedsSheppWkPtr_t& weak) { weak_ = weak; }

}  // namespace distance
}  //   namespace core
}  // namespace hpp
