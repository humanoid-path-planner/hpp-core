// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>

namespace bpt = boost::posix_time;

namespace hpp {
namespace core {
PathOptimizer::PathOptimizer(const ProblemConstPtr_t& problem)
    : interrupt_(false),
      problem_(problem),
      maxIterations_(std::numeric_limits<unsigned long int>::infinity()),
      timeOut_(std::numeric_limits<double>::infinity()) {
  monitor_.enabled = false;

  initFromParameters();
}

PathPtr_t PathOptimizer::steer(ConfigurationIn_t q1,
                               ConfigurationIn_t q2) const {
  PathPtr_t dp = (*problem()->steeringMethod())(q1, q2);
  if (dp) {
    if (!problem()->pathProjector()) return dp;
    PathPtr_t pp;
    if (problem()->pathProjector()->apply(dp, pp)) return pp;
  }
  return PathPtr_t();
}

void PathOptimizer::monitorExecution() {
  interrupt_ = false;
  monitor_.enabled = true;
  monitor_.iteration = 0;
  monitor_.timeStart = bpt::microsec_clock::universal_time();
}

bool PathOptimizer::shouldStop() const {
  if (interrupt_) return true;
  if (!monitor_.enabled) return false;
  if (monitor_.iteration >= maxIterations_) return true;

  bpt::ptime timeStop(bpt::microsec_clock::universal_time());
  if (static_cast<value_type>(
          (timeStop - monitor_.timeStart).total_milliseconds()) >
      timeOut_ * 1e3)
    return true;
  return false;
}

void PathOptimizer::initFromParameters() {
  maxIterations_ =
      problem()->getParameter("PathOptimizer/maxIterations").intValue();
  timeOut_ = problem()->getParameter("PathOptimizer/timeOut").floatValue();
}

void PathOptimizer::maxIterations(const unsigned long int& n) {
  maxIterations_ = n;
}

void PathOptimizer::timeOut(const double& timeOut) { timeOut_ = timeOut; }

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(PathOptimizer)
Problem::declareParameter(
    ParameterDescription(Parameter::INT, "PathOptimizer/maxIterations",
                         "Maximal number of iterations.",
                         Parameter(std::numeric_limits<size_type>::max())));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "PathOptimizer/timeOut",
    "Duration in seconds above which execution will stop."
    "The iteration at the moment the duration is elapsed will be completed.",
    Parameter(std::numeric_limits<double>::infinity())));
HPP_END_PARAMETER_DECLARATION(PathOptimizer)
}  // namespace core
}  // namespace hpp
