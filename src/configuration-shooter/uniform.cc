//
// Copyright (c) 2018 CNRS
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

#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace hpp {
namespace core {
namespace configurationShooter {

void Uniform::impl_shoot(Configuration_t& config) const {
  size_type extraDim = robot_->extraConfigSpace().dimension();
  size_type offset = robot_->configSize() - extraDim;

  config.resize(robot_->configSize());
  config.head(offset) = ::pinocchio::randomConfiguration(robot_->model());

  if (sampleExtraDOF_) {
    // Shoot extra configuration variables
    for (size_type i = 0; i < extraDim; ++i) {
      value_type lower = robot_->extraConfigSpace().lower(i);
      value_type upper = robot_->extraConfigSpace().upper(i);
      value_type range = upper - lower;
      if ((range < 0) || (range == std::numeric_limits<double>::infinity())) {
        std::ostringstream oss("Cannot uniformy sample extra config variable ");
        oss << i << ". min = " << lower << ", max = " << upper << std::endl;
        throw std::runtime_error(oss.str());
      }
      config[offset + i] = lower + (upper - lower) * rand() / RAND_MAX;
    }
  } else {
    config.tail(extraDim).setZero();
  }
}

}  // namespace configurationShooter
}  // namespace core
}  // namespace hpp
