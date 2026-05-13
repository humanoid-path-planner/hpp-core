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

#include <algorithm>
#include <cmath>
#include <hpp/core/path-optimization/trapezoidal-time-parameterization.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/time-parameterization.hh>
#include <limits>
#include <stdexcept>

namespace hpp {
namespace core {
namespace pathOptimization {

namespace {
class TrapezoidalProfile : public TimeParameterization {
 public:
  TrapezoidalProfile(const value_type s0, const value_type s1,
                     const value_type maxVelocity,
                     const value_type maxAcceleration,
                     const value_type minimumDuration) {
    if (maxVelocity <= 0 || !std::isfinite(maxVelocity)) {
      throw std::invalid_argument("maxVelocity should be positive.");
    }
    if (!std::isfinite(maxAcceleration)) {
      throw std::invalid_argument("maxAcceleration should be finite.");
    }

    times_.push_back(0);
    const value_type distance = s1 - s0;
    const value_type eps =
        std::sqrt(std::numeric_limits<value_type>::epsilon());
    if (distance <= eps) {
      addSegment(std::max(minimumDuration, eps), 0, 0, s0);
      return;
    }

    if (maxAcceleration <= 0) {
      const value_type duration =
          std::max(distance / maxVelocity, minimumDuration);
      const value_type velocity = distance / duration;
      peakVelocity_ = velocity;
      addSegment(duration, 0, velocity, s0);
      return;
    }

    value_type tacc = maxVelocity / maxAcceleration;
    value_type dacc = .5 * maxAcceleration * tacc * tacc;
    if (2 * dacc <= distance) {
      const value_type constantDuration = (distance - 2 * dacc) / maxVelocity;
      addSegment(tacc, .5 * maxAcceleration, 0, s0);
      addSegment(constantDuration, 0, maxVelocity, s0 + dacc);
      addSegment(tacc, -.5 * maxAcceleration, maxVelocity,
                 s0 + dacc + maxVelocity * constantDuration);
      peakVelocity_ = maxVelocity;
    } else {
      peakVelocity_ = std::sqrt(distance * maxAcceleration);
      tacc = peakVelocity_ / maxAcceleration;
      dacc = .5 * maxAcceleration * tacc * tacc;
      addSegment(tacc, .5 * maxAcceleration, 0, s0);
      addSegment(tacc, -.5 * maxAcceleration, peakVelocity_, s0 + dacc);
    }

    const value_type duration = times_.back();
    if (minimumDuration > duration && duration > eps) {
      scaleDuration(minimumDuration / duration);
    }
  }

  interval_t definitionInterval() const { return interval_t(0, times_.back()); }

  value_type value(const value_type& t) const {
    const size_type index = findInterval(t);
    const value_type dt = clampTime(t) - times_[(std::size_t)index];
    return a_[(std::size_t)index] * dt * dt + b_[(std::size_t)index] * dt +
           c_[(std::size_t)index];
  }

  value_type derivative(const value_type& t, const size_type& order) const {
    if (order == 0) return value(t);
    if (order > 2) return 0;

    const size_type index = findInterval(t);
    const value_type dt = clampTime(t) - times_[(std::size_t)index];
    if (order == 1)
      return 2 * a_[(std::size_t)index] * dt + b_[(std::size_t)index];
    return 2 * a_[(std::size_t)index];
  }

  value_type derivativeBound(const value_type& low,
                             const value_type& up) const {
    (void)low;
    (void)up;
    return peakVelocity_;
  }

  TimeParameterizationPtr_t copy() const {
    return TimeParameterizationPtr_t(new TrapezoidalProfile(*this));
  }

 private:
  void addSegment(const value_type duration, const value_type a,
                  const value_type b, const value_type c) {
    if (duration <= 0) return;
    a_.push_back(a);
    b_.push_back(b);
    c_.push_back(c);
    times_.push_back(times_.back() + duration);
  }

  void scaleDuration(const value_type scale) {
    for (std::size_t i = 1; i < times_.size(); ++i) times_[i] *= scale;
    for (std::size_t i = 0; i < a_.size(); ++i) {
      a_[i] /= scale * scale;
      b_[i] /= scale;
    }
    peakVelocity_ /= scale;
  }

  value_type clampTime(const value_type t) const {
    return std::min(std::max(t, times_.front()), times_.back());
  }

  size_type findInterval(const value_type t) const {
    const value_type tc = clampTime(t);
    for (std::size_t i = 1; i < times_.size(); ++i) {
      if (tc <= times_[i]) return (size_type)(i - 1);
    }
    return (size_type)(times_.size() - 2);
  }

  std::vector<value_type> times_;
  std::vector<value_type> a_, b_, c_;
  value_type peakVelocity_ = 0;
};  // class TrapezoidalProfile

value_type pathDerivativeBound(const PathPtr_t& path,
                               const interval_t& paramRange) {
  vector_t bound(path->outputDerivativeSize());
  bound.setZero();
  path->velocityBound(bound, paramRange.first, paramRange.second);
  const value_type result = bound.cwiseAbs().maxCoeff();
  if (!std::isfinite(result)) {
    throw std::runtime_error(
        "Invalid path derivative bound for TrapezoidalTimeParameterization.");
  }
  return result;
}
}  // namespace

TrapezoidalTimeParameterizationPtr_t TrapezoidalTimeParameterization::create(
    const ProblemConstPtr_t& problem) {
  TrapezoidalTimeParameterizationPtr_t ptr(
      new TrapezoidalTimeParameterization(problem));
  return ptr;
}

PathVectorPtr_t TrapezoidalTimeParameterization::optimize(
    const PathVectorPtr_t& path) {
  if (path->length() == 0) {
    return path;
  }

  PathVectorPtr_t input =
      PathVector::create(path->outputSize(), path->outputDerivativeSize());
  PathVectorPtr_t output =
      PathVector::create(path->outputSize(), path->outputDerivativeSize());
  path->flatten(input);

  const value_type eps = std::sqrt(std::numeric_limits<value_type>::epsilon());
  for (std::size_t i = 0; i < input->numberPaths(); ++i) {
    PathPtr_t p = input->pathAtRank(i);
    interval_t paramRange = p->paramRange();
    if (paramRange.second - paramRange.first <= eps) continue;

    p->timeParameterization(TimeParameterizationPtr_t(), paramRange);
    PathPtr_t pp = p->copy();

    const value_type derivativeBound = pathDerivativeBound(p, paramRange);
    value_type paramMaxVelocity = maxVelocity_;
    value_type paramMaxAcceleration = maxAcceleration_;
    if (derivativeBound > eps) {
      paramMaxVelocity /= derivativeBound;
      paramMaxAcceleration /= derivativeBound;
    }

    TrapezoidalProfile* profile = new TrapezoidalProfile(
        paramRange.first, paramRange.second, paramMaxVelocity,
        paramMaxAcceleration, minimumDuration_);
    TimeParameterizationPtr_t tp(profile);
    pp->timeParameterization(tp, profile->definitionInterval());
    output->appendPath(pp);
  }
  return output;
}

TrapezoidalTimeParameterization::TrapezoidalTimeParameterization(
    const ProblemConstPtr_t& problem)
    : PathOptimizer(problem),
      maxVelocity_(1.),
      maxAcceleration_(.5),
      minimumDuration_(.01) {}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
