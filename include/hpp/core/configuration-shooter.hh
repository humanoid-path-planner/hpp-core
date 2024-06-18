//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_HH
#define HPP_CORE_CONFIGURATION_SHOOTER_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup configuration_sampling
/// \{

/// Abstraction of configuration shooter
///
/// Configuration shooters are used by random sampling algorithms to
/// generate new configurations
class HPP_CORE_DLLAPI ConfigurationShooter {
 public:
  /// Shoot a random configuration
  virtual Configuration_t shoot() const {
    Configuration_t q;
    shoot(q);
    return q;
  }

  /// Shoot a random configuration
  /// \param q the configuration (resized if necessary).
  ///
  /// \deprecated This method is virtual for backward compatibility. It will
  /// become non-virtual in the future. Child classes should rather implement
  /// \ref impl_shoot so that both prototype of method shoot remain available.
  virtual void shoot(Configuration_t& q) const { impl_shoot(q); }

  virtual ~ConfigurationShooter() {};

 protected:
  ConfigurationShooter() {}
  /// Store weak pointer to itself
  void init(const ConfigurationShooterWkPtr_t& weak) { weakPtr_ = weak; }

  virtual void impl_shoot(Configuration_t& q) const = 0;

 private:
  ConfigurationShooterWkPtr_t weakPtr_;
};  // class
}  //   namespace core
/// \}
}  // namespace hpp
#endif  // HPP_CORE_CONFIGURATION_SHOOTER_HH
