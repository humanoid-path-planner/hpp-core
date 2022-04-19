// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_HERMITE_HH
#define HPP_CORE_STEERING_METHOD_HERMITE_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method/fwd.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
namespace core {
namespace steeringMethod {
/// \addtogroup steering_method
/// \{

/// Steering method that creates path::Hermite instances
///
class HPP_CORE_DLLAPI Hermite : public SteeringMethod {
 public:
  /// Create instance and return shared pointer
  static HermitePtr_t create(const ProblemConstPtr_t& problem) {
    Hermite* ptr = new Hermite(problem);
    HermitePtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  static HermitePtr_t createCopy(const HermitePtr_t& other) {
    Hermite* ptr = new Hermite(*other);
    HermitePtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  virtual SteeringMethodPtr_t copy() const { return createCopy(weak_.lock()); }

  /// create a path between two configurations
  virtual PathPtr_t impl_compute(ConfigurationIn_t q1,
                                 ConfigurationIn_t q2) const {
    path::HermitePtr_t path =
        path::Hermite::create(problem()->robot(), q1, q2, constraints());

    path->computeHermiteLength();
    return path;
  }

 protected:
  /// Constructor with weighed distance
  Hermite(const ProblemConstPtr_t& problem)
      : SteeringMethod(problem), weak_() {}

  /// Copy constructor
  Hermite(const Hermite& other) : SteeringMethod(other), weak_() {}

  /// Store weak pointer to itself
  void init(HermiteWkPtr_t weak) {
    SteeringMethod::init(weak);
    weak_ = weak;
  }

 private:
  HermiteWkPtr_t weak_;
};  // Hermite
/// \}
}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_STEERING_METHOD_HERMITE_HH
