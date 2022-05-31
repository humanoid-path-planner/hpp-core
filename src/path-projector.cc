// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include "hpp/core/path-projector.hh"

#include <hpp/core/distance.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/util/pointer.hh>
#include <hpp/util/timer.hh>

namespace hpp {
namespace core {
namespace {
HPP_DEFINE_TIMECOUNTER(PathProjection);
}

PathProjector::PathProjector(const DistancePtr_t& distance,
                             const SteeringMethodPtr_t& steeringMethod,
                             bool keepSteeringMethodConstraints)
    : steeringMethod_(steeringMethod->copy()), distance_(distance) {
  assert(distance_ != NULL);
  assert(steeringMethod_ != NULL);
  if (!keepSteeringMethodConstraints) {
    steeringMethod_->constraints(ConstraintSetPtr_t());
  }
}

PathProjector::~PathProjector() {
  HPP_DISPLAY_TIMECOUNTER(PathProjection);
  HPP_RESET_TIMECOUNTER(PathProjection);
}

value_type PathProjector::d(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
  return (*distance_)(q1, q2);
}

PathPtr_t PathProjector::steer(ConfigurationIn_t q1,
                               ConfigurationIn_t q2) const {
  PathPtr_t result((*steeringMethod_)(q1, q2));
  // In the case of hermite path, we want the paths to be constrained.
  // assert (!result->constraints ());
  return result;
}

bool PathProjector::apply(const PathPtr_t& path, PathPtr_t& proj) const {
  HPP_START_TIMECOUNTER(PathProjection);
  bool ret = impl_apply(path, proj);
  HPP_STOP_TIMECOUNTER(PathProjection);
  return ret;
}

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(pathProjection)
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "PathProjection/HessianBound",
                         "A bound on the norm of the hessian of the "
                         "constraints. Not considered if negative.",
                         Parameter(-1.)));
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "PathProjection/MinimalDist",
                         "The threshold which stops the projection (distance "
                         "between consecutive interpolation points.)",
                         Parameter(1e-3)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "PathProjection/RecursiveHermite/Beta",
    "See \"Fast Interpolation and Time-Optimization on Implicit Contact "
    "Submanifolds\" from Kris Hauser.",
    Parameter(0.9)));
HPP_END_PARAMETER_DECLARATION(pathProjection)
}  // namespace core
}  // namespace hpp
