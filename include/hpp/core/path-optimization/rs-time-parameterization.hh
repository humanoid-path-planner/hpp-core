//
// Copyright (c) 2019 - 2024 CNRS
//
// Author: Florent Lamiraux
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_RS_TIME_PARAMETERIZATION_HH
#define  HPP_CORE_PATH_OPTIMIZATION_RS_TIME_PARAMETERIZATION_HH

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/steering-method/constant-curvature.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
  /// Time parameterization of Reeds and Shepp paths
  ///
  /// This class computes a time parameterization for a concatentation
  /// of Reeds and Shepp paths (in fact of constant curvature paths)
  class HPP_CORE_DLLAPI RSTimeParameterization : public PathOptimizer
  {
  public:
    /// Create instance and return shared pointer
    static RSTimeParameterizationPtr_t create(const ProblemConstPtr_t& problem);
    /// Compute the time parameterization of a path
    ///
    /// \param path input path,
    /// \precond path should be a concatenation of
    ///          hpp::core::ConstantCurvature instances
    ///
    /// The parameterized path starts and ends with velocity equal to 0.
    /// On each segment, it accelerates, moves at maximal speed and
    /// decelerates to the minimal velocity to pass curvature discontinuity.
    virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);
  protected:
    /// Constructor
    /// \param minLinVel, maxLinVel minimal and maximal velocity.
    ///        Minimal velocity is the linear velocity allowed when
    ///        switching discontinuously the radius of curvature,
    /// \param maxAngVel maximal angular velocity,
    /// \param linearAcceletion linearDeceleration linear acceleration and
    ///        deceleration that can be different.
    RSTimeParameterization(const ProblemConstPtr_t& problem) : PathOptimizer(problem) {}
  }; // class ReedsShepp

} // namespace pathOptimization
} // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_RS_TIME_PARAMETERIZATION_HH
