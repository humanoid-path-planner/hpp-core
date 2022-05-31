//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#include <cstdlib>
#include <deque>
#include <hpp/core/distance.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <limits>

namespace hpp {
namespace core {
namespace pathOptimization {
// Compute the length of a vector of paths assuming that each element
// is optimal for the given distance.
template <bool reEstimateLength = false>
struct _PathLength {
  static inline value_type run(const PathVectorPtr_t& path,
                               const DistancePtr_t& distance) {
    if (reEstimateLength)
      return path->length();
    else {
      value_type result = 0;
      for (std::size_t i = 0; i < path->numberPaths(); ++i) {
        const PathPtr_t& element(path->pathAtRank(i));
        Configuration_t q1 = element->initial();
        Configuration_t q2 = element->end();
        result += (*distance)(q1, q2);
      }
      return result;
    }
  }
};

RandomShortcutPtr_t RandomShortcut::create(const ProblemConstPtr_t& problem) {
  RandomShortcut* ptr = new RandomShortcut(problem);
  return RandomShortcutPtr_t(ptr);
}

RandomShortcut::RandomShortcut(const ProblemConstPtr_t& problem)
    : PathOptimizer(problem) {}

PathVectorPtr_t RandomShortcut::optimize(const PathVectorPtr_t& path) {
  monitorExecution();

  using std::make_pair;
  using std::numeric_limits;
  bool finished = false;
  value_type t[4];
  Configuration_t q[4];
  q[0] = path->initial();
  q[1].resize(path->outputSize()), q[2].resize(path->outputSize());
  q[3] = path->end();
  PathVectorPtr_t tmpPath = path;

  // Maximal number of iterations without improvements
  const std::size_t n =
      problem()
          ->getParameter("PathOptimization/RandomShortcut/NumberOfLoops")
          .intValue();
  std::size_t projectionError = n;
  std::deque<value_type> length(n - 1, numeric_limits<value_type>::infinity());
  length.push_back(_PathLength<>::run(tmpPath, problem()->distance()));
  PathVectorPtr_t result;

  while (!shouldStop() && !finished && projectionError != 0) {
    endIteration();
    t[0] = tmpPath->timeRange().first;
    t[3] = tmpPath->timeRange().second;
    bool error = !shootTimes(tmpPath, t[0], t[1], t[2], t[3]);
    if (error) {
      projectionError--;
      continue;
    }
    for (int i = 1; i < 3; ++i) {
      if (!(*tmpPath)(q[i], t[i])) {
        hppDout(error,
                "Configuration at param " << t[i] << " could not be projected");
        projectionError--;
        error = true;
        break;
      }
    }
    if (error) continue;
    // Validate sub parts
    bool valid[3];
    PathPtr_t proj[3];
    // Build and projects the path
    for (int i = 0; i < 3; ++i) proj[i] = steer(q[i], q[i + 1]);
    if (!proj[0] && !proj[1] && !proj[2]) {
      hppDout(info, "Enable to create a valid path");
      projectionError--;
      continue;
    }
    // validate the paths
    for (unsigned i = 0; i < 3; ++i) {
      PathPtr_t validPart;
      PathValidationReportPtr_t report;
      if (!proj[i])
        valid[i] = false;
      else
        valid[i] = problem()->pathValidation()->validate(proj[i], false,
                                                         validPart, report);
    }
    // Replace valid parts
    result =
        PathVector::create(path->outputSize(), path->outputDerivativeSize());
    try {
      for (int i = 0; i < 3; ++i) {
        if (valid[i])
          result->appendPath(proj[i]);
        else
          result->concatenate(
              tmpPath->extract(make_pair(t[i], t[i + 1]))->as<PathVector>());
      }
    } catch (const projection_error& e) {
      hppDout(error, "Caught exception at with time " << t[1] << " and " << t[2]
                                                      << ": " << e.what());
      projectionError--;
      result = tmpPath;
      continue;
    }
    value_type newLength = _PathLength<>::run(result, problem()->distance());
    if (length[n - 1] <= newLength) {
      hppDout(info, "the length would increase:" << length[n - 1] << " "
                                                 << newLength);
      result = tmpPath;
      projectionError--;
    } else {
      length.push_back(newLength);
      length.pop_front();
      finished = (length[0] - length[n - 1]) <= 1e-4 * length[n - 1];
      hppDout(info, "length = " << length[n - 1]);
      tmpPath = result;
      projectionError = n;
    }
  }
  if (!result) return path;
  hppDout(info, "RandomShortcut:" << *result);
  for (std::size_t i = 0; i < result->numberPaths(); ++i) {
    if (result->pathAtRank(i)->constraints())
      hppDout(info, "At rank " << i << ", constraints are "
                               << *result->pathAtRank(i)->constraints());
    else
      hppDout(info, "At rank " << i << ", no constraints");
  }
  return result;
}

bool RandomShortcut::shootTimes(const PathVectorPtr_t& /*current*/,
                                const value_type& t0, value_type& t1,
                                value_type& t2, const value_type& t3) {
  value_type u2 = (t3 - t0) * rand() / RAND_MAX;
  value_type u1 = (t3 - t0) * rand() / RAND_MAX;
  if (u1 < u2) {
    t1 = t0 + u1;
    t2 = t0 + u2;
  } else {
    t1 = t0 + u2;
    t2 = t0 + u1;
  }
  return true;
}

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(RandomShortcut)
Problem::declareParameter(ParameterDescription(
    Parameter::INT, "PathOptimization/RandomShortcut/NumberOfLoops",
    "Number of loops.", Parameter((size_type)5)));
HPP_END_PARAMETER_DECLARATION(RandomShortcut)
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
