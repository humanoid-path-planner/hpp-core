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

#ifndef HPP_CORE_STEERING_METHOD_HH
#define HPP_CORE_STEERING_METHOD_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
/// \addtogroup steering_method
/// \{

/// Steering method
///
/// A steering method creates paths between pairs of
/// configurations for a robot. They are usually used to take
/// into account nonholonomic constraints of some robots
class HPP_CORE_DLLAPI SteeringMethod {
 public:
  /// create a path between two configurations
  /// \return a Path from q1 to q2 if found. An empty
  /// Path if Path could not be built.
  /// \note if q1 == q2, the steering method should not return an empty
  ///       shared pointer.
  PathPtr_t operator()(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
    PathPtr_t path;
    try {
      path = impl_compute(q1, q2);
    } catch (const projection_error& e) {
      hppDout(info, "Could not build path: " << e.what());
    }
    assert(q1 != q2 || path);
    return path;
  }

  /// \copydoc SteeringMethod::operator()(ConfigurationIn_t,ConfigurationIn_t)
  PathPtr_t steer(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
    return this->operator()(q1, q2);
  }

  virtual ~SteeringMethod() {};

  /// Copy instance and return shared pointer
  virtual SteeringMethodPtr_t copy() const = 0;

  ProblemConstPtr_t problem() const { return problem_.lock(); }

  /// \name Constraints applicable to the robot.
  /// These constraints are not automatically taken into
  /// account. Child class can use it if they need.
  /// \{

  /// Set constraint set
  void constraints(const ConstraintSetPtr_t& constraints) {
    constraints_ = constraints;
  }

  /// Get constraint set
  const ConstraintSetPtr_t& constraints() const { return constraints_; }
  /// \}

 protected:
  /// Constructor
  SteeringMethod(const ProblemConstPtr_t& problem)
      : problem_(problem), constraints_(), weak_() {}
  /// Copy constructor
  ///
  /// Constraints are copied
  SteeringMethod(const SteeringMethod& other)
      : problem_(other.problem_), constraints_(), weak_() {
    if (other.constraints_) {
      constraints_ =
          HPP_DYNAMIC_PTR_CAST(ConstraintSet, other.constraints_->copy());
    }
  }
  /// create a path between two configurations
  virtual PathPtr_t impl_compute(ConfigurationIn_t q1,
                                 ConfigurationIn_t q2) const = 0;
  /// Store weak pointer to itself.
  void init(SteeringMethodWkPtr_t weak) { weak_ = weak; }

 private:
  ProblemConstWkPtr_t problem_;
  /// Set of constraints to apply on the paths produced
  ConstraintSetPtr_t constraints_;
  /// Weak pointer to itself
  SteeringMethodWkPtr_t weak_;
};  // class SteeringMethod
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_STEERING_METHOD_HH
