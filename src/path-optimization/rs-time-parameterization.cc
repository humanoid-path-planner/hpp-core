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

#include <hpp/core/path-optimization/rs-time-parameterization.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <path-optimization/reeds-shepp/piecewise-quadratic.hh>

namespace hpp {
namespace core {
namespace pathOptimization {

RSTimeParameterizationPtr_t RSTimeParameterization::create(
    const ProblemConstPtr_t& problem) {
  RSTimeParameterization* pointer(new RSTimeParameterization(problem));
  return RSTimeParameterizationPtr_t(pointer);
}

PathVectorPtr_t RSTimeParameterization::optimize(const PathVectorPtr_t& path) {
  using reedsShepp::PiecewiseQuadratic;
  using reedsShepp::PiecewiseQuadraticPtr_t;
  value_type minLinVel_(
      problem()
          ->getParameter("RSTimeParameterization/MinLinearVelocity")
          .floatValue());
  value_type maxLinVel_(
      problem()
          ->getParameter("RSTimeParameterization/MaxLinearVelocity")
          .floatValue());
  value_type maxAngVel_(
      problem()
          ->getParameter("RSTimeParameterization/MaxAngularVelocity")
          .floatValue());
  value_type linearAcceleration_(
      problem()
          ->getParameter("RSTimeParameterization/LinearAcceleration")
          .floatValue());
  value_type linearDeceleration_(
      problem()
          ->getParameter("RSTimeParameterization/LinearDeceleration")
          .floatValue());

  // Retrieve parameters from problem

  value_type eps(sqrt(std::numeric_limits<value_type>::epsilon()));
  PathVectorPtr_t flattenedPath(
      PathVector::create(path->outputSize(), path->outputDerivativeSize()));
  // Flatten input path in case it contains PathVector instances with
  // ConstantCurvature instances.
  path->flatten(flattenedPath);
  PathVectorPtr_t result(
      PathVector::create(path->outputSize(), path->outputDerivativeSize()));

  // Start with zero velocity and end first segment with the
  // minimal velocity
  value_type initVel(0), targetVel(minLinVel_), maxLinVel;
  value_type linearAcceleration, linearDeceleration;
  // Loop over each constant curvature segment
  for (std::size_t i = 0; i < flattenedPath->numberPaths(); ++i) {
    // if last segment, end with 0 velocity
    targetVel = 0;
    PathPtr_t p(flattenedPath->pathAtRank(i)->copy());
    if (p->length() > eps) {
      value_type t0(p->timeRange().second);
      vector_t v0(p->outputDerivativeSize());
      p->derivative(v0, t0, 1);
      if (i != flattenedPath->numberPaths() - 1) {
        // If two consecutive segments are in the same direction (i.e.
        // forward or backward), decelerate to minLinVel_
        PathPtr_t next(flattenedPath->pathAtRank(i + 1));
        value_type t1(next->timeRange().first);
        vector_t v1(next->outputDerivativeSize());
        next->derivative(v1, t1, 1);
        if ((v1 - v0).head<2>().norm() < 1e-3) {
          targetVel = minLinVel_;
        }
      }
      // Compute maximal linear velocity
      value_type curvature(fabs(v0[2]));
      hppDout(info, "curvature=" << curvature);
      if (curvature * maxLinVel_ < maxAngVel_) {
        // Sature linear velocity
        maxLinVel = maxLinVel_;
        linearAcceleration = linearAcceleration_;
        linearDeceleration = linearDeceleration_;
      } else {
        // Saturate angular velocity
        maxLinVel = maxAngVel_ / curvature;
        // reduce accelerations by the same ratio
        // We could also add maximal angular acceleration parameter
        linearAcceleration = linearAcceleration_ * maxLinVel / maxLinVel_;
        linearDeceleration = linearDeceleration_ * maxLinVel / maxLinVel_;
      }
      // Create one parameterization per segment
      PiecewiseQuadraticPtr_t param(PiecewiseQuadratic::create(initVel));
      param->addSegments(p->length(), linearAcceleration, linearDeceleration,
                         maxLinVel, targetVel);
      p->timeParameterization(param, param->definitionInterval());
      result->appendPath(p);
      // From second segment, start at minimal velocity
      initVel = minLinVel_;
    }
  }
  return result;
}
// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(RSTimeParameterization)
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "RSTimeParameterization/MinLinearVelocity",
    "Maximal linear velocity allowed when crossing a curvature discontinuity",
    Parameter(.1)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "RSTimeParameterization/MaxLinearVelocity",
    "Maximal linear velocity", Parameter(1.)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "RSTimeParameterization/MaxAngularVelocity",
    "Maximal angular velocity", Parameter(1.)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "RSTimeParameterization/LinearAcceleration",
    "Linear acceleration", Parameter(1.)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "RSTimeParameterization/LinearDeceleration",
    "Linear deceleration", Parameter(1.)));
HPP_END_PARAMETER_DECLARATION(RSTimeParameterization)
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
