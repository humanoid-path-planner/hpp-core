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

#ifndef HPP_CORE_CONSTRAINT_HH
#define HPP_CORE_CONSTRAINT_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/util/serialization-fwd.hh>
#include <ostream>

namespace hpp {
namespace core {
/// \addtogroup constraints
/// \{

/// Constraint applicable to a robot configuration
///
/// Constraint::apply takes as input a configuration and changes it into
/// a configuration satisfying the constraint.
///
/// User should define impl_compute in derived classes.
class HPP_CORE_DLLAPI Constraint {
 public:
  /// Function that applies the constraint
  /// \param configuration initial configuration and result
  /// \return true if constraint applied successfully, false if failure.
  bool apply(ConfigurationOut_t configuration);
  /// Get name of constraint
  const std::string& name() const { return name_; }

  /// Check whether a configuration statisfies the constraint.
  ///
  /// \param config the configuration to check
  virtual bool isSatisfied(ConfigurationIn_t config) = 0;

  /// Check whether a configuration statisfies the constraint.
  ///
  /// \param config the configuration to check
  /// \retval error error expressed as a vector. Size and content depends
  ///         on implementations
  virtual bool isSatisfied(ConfigurationIn_t config, vector_t& error) = 0;

  /// return shared pointer to copy
  virtual ConstraintPtr_t copy() const = 0;

  virtual ~Constraint() {};

 protected:
  /// User defined implementation of the constraint.
  virtual bool impl_compute(ConfigurationOut_t configuration) = 0;
  /// Constructor
  Constraint(const std::string& name) : name_(name), weak_() {}
  Constraint(const Constraint& constraint) : name_(constraint.name_), weak_() {}
  /// Store shared pointer to itself
  void init(const ConstraintPtr_t& self) { weak_ = self; }

 private:
  virtual std::ostream& print(std::ostream& os) const = 0;

  virtual void addLockedJoint(const LockedJointPtr_t&) {}

  std::string name_;
  ConstraintWkPtr_t weak_;
  friend class ConstraintSet;
  friend class constraints::LockedJoint;
  friend class ConfigProjector;
  friend std::ostream& operator<<(std::ostream& os, const Constraint&);

  Constraint() {}
  HPP_SERIALIZABLE();
};  // class Constraint
inline std::ostream& operator<<(std::ostream& os,
                                const Constraint& constraint) {
  return constraint.print(os);
}
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONSTRAINT_HH
