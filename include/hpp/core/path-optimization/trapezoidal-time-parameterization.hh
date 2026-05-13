// Copyright (c) 2026 CNRS
// Author: Paul Sardin
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_TRAPEZOIDAL_TIME_PARAMETERIZATION_HH
#define HPP_CORE_PATH_OPTIMIZATION_TRAPEZOIDAL_TIME_PARAMETERIZATION_HH

#include <hpp/core/path-optimizer.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
/// \addtogroup path_optimization
/// \{

/// Add a trapezoidal velocity profile time parameterization to a path.
///
/// The optimizer computes one scalar time law per direct subpath. The maximum
/// scalar path velocity and acceleration are chosen so that, using the path
/// velocity bound, no output degree of freedom exceeds the configured scalar
/// limits.
///
/// Parameters can be configured with the getter and setter methods:
/// \li maxVelocity
/// \li maxAcceleration
/// \li minimumDuration
class HPP_CORE_DLLAPI TrapezoidalTimeParameterization : public PathOptimizer {
 public:
  /// Return shared pointer to new object.
  static TrapezoidalTimeParameterizationPtr_t create(
      const ProblemConstPtr_t& problem);

  /// Optimize path.
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

  /// Maximum velocity for each output degree of freedom.
  value_type maxVelocity() const { return maxVelocity_; }
  /// Set maximum velocity for each output degree of freedom.
  void maxVelocity(const value_type& maxVelocity) {
    maxVelocity_ = maxVelocity;
  }

  /// Maximum acceleration for each output degree of freedom.
  value_type maxAcceleration() const { return maxAcceleration_; }
  /// Set maximum acceleration for each output degree of freedom.
  void maxAcceleration(const value_type& maxAcceleration) {
    maxAcceleration_ = maxAcceleration;
  }

  /// Minimum duration assigned to each non-empty subpath.
  value_type minimumDuration() const { return minimumDuration_; }
  /// Set minimum duration assigned to each non-empty subpath.
  void minimumDuration(const value_type& minimumDuration) {
    minimumDuration_ = minimumDuration;
  }

 protected:
  TrapezoidalTimeParameterization(const ProblemConstPtr_t& problem);

 private:
  value_type maxVelocity_;
  value_type maxAcceleration_;
  value_type minimumDuration_;
};  // class TrapezoidalTimeParameterization
/// \}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_TRAPEZOIDAL_TIME_PARAMETERIZATION_HH
