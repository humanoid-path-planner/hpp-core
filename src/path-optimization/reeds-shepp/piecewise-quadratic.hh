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

#ifndef HPP_CORE_PATH_OPTIMIZATION_REEDS_SHEPP_PIECEWISE_QUADRATIC_HH
#define HPP_CORE_PATH_OPTIMIZATION_REEDS_SHEPP_PIECEWISE_QUADRATIC_HH

#include <hpp/core/time-parameterization.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
namespace reedsShepp {

using hpp::core::interval_t;
HPP_PREDEF_CLASS(PiecewiseQuadratic);
typedef hpp::shared_ptr<PiecewiseQuadratic> PiecewiseQuadraticPtr_t;

/// Piecewise quadratic time parameterization
///
/// On each interval \f$[t_i,t_{i+1}], f (t) = a_i (t-t_i) + b_i(t-t_i) +
/// c_i^2\f$.
class PiecewiseQuadratic : public hpp::core::TimeParameterization {
 public:
  /// Create an instance
  /// \param initVel initial velocity
  static PiecewiseQuadraticPtr_t create(const value_type& initVel);
  static PiecewiseQuadraticPtr_t createCopy(
      const PiecewiseQuadraticPtr_t& other);
  virtual hpp::core::TimeParameterizationPtr_t copy() const;
  interval_t definitionInterval() const;
  virtual value_type value(const value_type& t) const;
  virtual value_type derivative(const value_type& t,
                                const size_type& order) const;
  virtual value_type derivativeBound(const value_type& low,
                                     const value_type& up) const;
  /// Add up to 3 constant acceleration segments
  ///
  /// \param distance distance travelled on these segments,
  /// \param accel constant acceleration on the first segment,
  /// \param decel constant negative acceleration on the last segment,
  /// \param maxVel constant velocity on the middle segment
  /// \param targetVel velocity at the end of the segment
  /// \note depending on the distance, one of the above segment might be
  ///       of size 0, and therefore not represented.
  void addSegments(const value_type& distance, const value_type& accel,
                   const value_type& decel, const value_type& maxVel,
                   const value_type& targetVel);

 protected:
  PiecewiseQuadratic(const value_type& initVel) : initVel_(initVel) {
    times_.push_back(0);
  }

  PiecewiseQuadratic(const PiecewiseQuadratic& other)
      : hpp::core::TimeParameterization(other),
        times_(other.times_),
        a_(other.a_),
        b_(other.b_),
        c_(other.c_),
        initVel_(other.initVel_) {}
  void init(const PiecewiseQuadraticWkPtr_t& weak);

 private:
  size_type findInterval(value_type t) const;
  std::vector<value_type> times_;
  std::vector<value_type> a_, b_, c_;
  value_type initVel_;
  PiecewiseQuadraticWkPtr_t weak_;
};  // class PiecewiseQuadratic
}  // namespace reedsShepp
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_REEDS_SHEPP_PIECEWISE_QUADRATIC_HH
