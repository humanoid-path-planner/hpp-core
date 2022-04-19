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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH
#define HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH

#include <hpp/core/configuration-shooter.hh>
#include <hpp/pinocchio/device.hh>
#include <sstream>

namespace hpp {
namespace core {
namespace configurationShooter {
/// \addtogroup configuration_sampling
/// \{

/// Sample configuration using a gaussian distribution around a
/// configuration.
class HPP_CORE_DLLAPI Gaussian : public ConfigurationShooter {
 public:
  static GaussianPtr_t create(const DevicePtr_t& robot) {
    Gaussian* ptr = new Gaussian(robot);
    GaussianPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  void center(ConfigurationIn_t c) { center_ = c; }
  const Configuration_t& center() const { return center_; }

  /// Set the standard deviation proportional to a default value
  ///
  /// The default value is:
  /// \li for vector spaces, the difference between the upper and the
  ///     lower bounds,
  /// \li for SO(n), \f$\frac{2\pi}{\sqrt{2n-3}}\f$ on each of the
  ///     \f$ 2n-3 \f$ dimensions,
  /// \li SE(n) is treated as \f$ R^n \times SO(n) \f$.
  void sigma(const value_type& factor);

  void sigmas(vectorIn_t s) {
    assert(s.size() == robot_->numberDof());
    sigmas_ = s;
  }
  const vector_t& sigmas() const { return sigmas_; }

 protected:
  /// Create a gaussian distribution centered in the robot current
  /// configuration. The standard deviation is computed as \c sigma(0.25)
  /// \sa Gaussian::sigma
  Gaussian(const DevicePtr_t& robot)
      : robot_(robot),
        center_(robot->currentConfiguration()),
        sigmas_(robot->numberDof()) {
    sigma(1. / 4.);
  }
  void init(const GaussianPtr_t& self) {
    ConfigurationShooter::init(self);
    weak_ = self;
  }

  virtual void impl_shoot(Configuration_t& q) const;

 private:
  const DevicePtr_t& robot_;
  /// The mean value
  Configuration_t center_;
  /// The standard deviation
  vector_t sigmas_;

  GaussianWkPtr_t weak_;
};  // class Gaussian
/// \}
}  //   namespace configurationShooter
}  //   namespace core
}  // namespace hpp

#endif  // HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH
