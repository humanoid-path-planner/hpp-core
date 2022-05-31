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

#include <boost/algorithm/string.hpp>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/car-like.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/joint/joint-generic.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace core {
namespace steeringMethod {
CarLike::CarLike(const ProblemConstPtr_t& problem)
    : SteeringMethod(problem),
      device_(problem->robot()),
      rho_(1.),
      xyId_(0),
      rzId_(2) {
  DevicePtr_t d(device_.lock());
  xy_ = d->getJointAtConfigRank(0);
  rz_ = d->getJointAtConfigRank(2);
  wheels_ = getWheelsFromParameter(problem, rz_);
  turningRadius(problem->getParameter("SteeringMethod/Carlike/turningRadius")
                    .floatValue());
}

CarLike::CarLike(const ProblemConstPtr_t& problem,
                 const value_type turningRadius, JointPtr_t xyJoint,
                 JointPtr_t rzJoint, std::vector<JointPtr_t> wheels)
    : SteeringMethod(problem),
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

/// Copy constructor
CarLike::CarLike(const CarLike& other)
    : SteeringMethod(other),
      device_(other.device_),
      rho_(other.rho_),
      xy_(other.xy_),
      rz_(other.rz_),
      xyId_(other.xyId_),
      rzId_(other.rzId_) {}

void CarLike::turningRadius(const value_type& rho) {
  if (rho <= 0)
    throw std::invalid_argument("Turning radius must be strictly positive.");
  rho_ = rho;
}

std::vector<JointPtr_t> getWheelsFromParameter(const ProblemConstPtr_t& problem,
                                               const JointPtr_t& rz) {
  std::vector<JointPtr_t> wheels;
  std::string p(
      problem->getParameter("SteeringMethod/Carlike/wheels").stringValue());
  std::vector<std::string> wheelNames;
  boost::split(wheelNames, p, [](char c) { return c == ','; });

  wheels.clear();
  for (const std::string& name : wheelNames) {
    if (name == "") continue;
    bool found(false);
    for (std::size_t i = 0; i < rz->numberChildJoints(); ++i) {
      JointPtr_t j = rz->childJoint(i);
      if (j->name() == name) {
        found = true;
        if (j->configSize() != 1) {
          throw std::runtime_error(
              "Carlike: wheel joint should be of dimension 1.");
        }
        wheels.push_back(j);
        hppDout(info, "wheel: " << name);
      }
    }
    if (!found) {
      std::ostringstream os;
      os << "CarLike: no joint with name \"" << name << "\".";
      throw std::runtime_error(os.str());
    }
  }
  return wheels;
}

}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
