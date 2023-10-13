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

#include <hpp/core/config-projector.hh>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
namespace path {
Hermite::Hermite(const DevicePtr_t& device, ConfigurationIn_t init,
                 ConfigurationIn_t end, ConstraintSetPtr_t constraints)
    : parent_t(device, interval_t(0, 1), constraints),
      init_(init),
      end_(end),
      hermiteLength_(-1) {
  assert(init.size() == robot_->configSize());
  assert(device);

  base(init);
  parameters_.row(0).setZero();
  pinocchio::difference<hpp::pinocchio::RnxSOnLieGroupMap>(robot_, end, init,
                                                           parameters_.row(3));

  projectVelocities(init, end);
}

Hermite::Hermite(const Hermite& path)
    : parent_t(path), init_(path.init_), end_(path.end_), hermiteLength_(-1) {}

Hermite::Hermite(const Hermite& path, const ConstraintSetPtr_t& constraints)
    : parent_t(path, constraints),
      init_(path.init_),
      end_(path.end_),
      hermiteLength_(-1) {
  projectVelocities(init_, end_);
}

void Hermite::init(HermitePtr_t self) {
  parent_t::init(self);
  weak_ = self;
  checkPath();
}

// void Hermite::computeVelocities (ConfigurationIn_t qi, ConfigurationIn_t qe)
void Hermite::projectVelocities(ConfigurationIn_t qi, ConfigurationIn_t qe) {
  // vector_t v_i2e (outputDerivativeSize());
  // pinocchio::difference<hpp::pinocchio::LieGroupTpl> (robot_, qe, qi, v_i2e);
  if (constraints() && constraints()->configProjector()) {
    ConfigProjectorPtr_t proj = constraints()->configProjector();
    vector_t v(outputDerivativeSize());
    // Compute v0
    // proj->projectVectorOnKernel (qi, v_i2e, v);
    proj->projectVectorOnKernel(qi, parameters_.row(3), v);
    v0(v);
    // Compute v1
    proj->projectVectorOnKernel(qe, parameters_.row(3), v);
    v1(v);
  } else {
    v0(parameters_.row(3));
    v1(parameters_.row(3));
  }
  assert(!parameters_.hasNaN());
}

void Hermite::computeHermiteLength() {
  hermiteLength_ = (parameters_.bottomRows<3>() - parameters_.topRows<3>())
                       .rowwise()
                       .norm()
                       .sum();
}

vector_t Hermite::velocity(const value_type& param) const {
  vector_t v(outputDerivativeSize());
  derivative(v, param, 1);
  if (constraints() && constraints()->configProjector()) {
    ConfigProjectorPtr_t proj = constraints()->configProjector();
    Configuration_t q(outputSize());
    if (!eval(q, param))
      throw projection_error("Configuration does not satisfy the constraints");
    proj->projectVectorOnKernel(q, v, v);
  }
  return v;
}

DevicePtr_t Hermite::device() const { return robot_; }
}  //   namespace path
}  //   namespace core
}  // namespace hpp
