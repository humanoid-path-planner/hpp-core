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

#include "hpp/core/path-projector/progressive.hh"

#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/util/timer.hh>

// TODO used to access parameters of the problem. We should not do this here.
//      See todo of pathProjector::Global::create
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method/straight.hh>
#include <limits>
#include <queue>
#include <stack>

namespace hpp {
namespace core {
namespace pathProjector {
constraints::solver::lineSearch::Constant lineSearch;

ProgressivePtr_t Progressive::create(const DistancePtr_t& distance,
                                     const SteeringMethodPtr_t& steeringMethod,
                                     value_type step) {
  value_type hessianBound = steeringMethod->problem()
                                ->getParameter("PathProjection/HessianBound")
                                .floatValue();
  value_type thr_min = steeringMethod->problem()
                           ->getParameter("PathProjection/MinimalDist")
                           .floatValue();
  hppDout(info, "Hessian bound is " << hessianBound);
  hppDout(info, "Min Dist is " << thr_min);
  return ProgressivePtr_t(
      new Progressive(distance, steeringMethod, step, thr_min, hessianBound));
}

ProgressivePtr_t Progressive::create(const ProblemConstPtr_t& problem,
                                     const value_type& step) {
  return create(problem->distance(), problem->steeringMethod(), step);
}

Progressive::Progressive(const DistancePtr_t& distance,
                         const SteeringMethodPtr_t& steeringMethod,
                         value_type step, value_type thresholdMin,
                         value_type hessianBound)
    : PathProjector(distance, steeringMethod),
      step_(step),
      thresholdMin_(thresholdMin),
      hessianBound_(hessianBound),
      withHessianBound_(hessianBound > 0) {
  steeringMethod::StraightPtr_t sm(
      HPP_DYNAMIC_PTR_CAST(steeringMethod::Straight, steeringMethod));
  if (!sm)
    throw std::logic_error(
        "The steering method should be of type"
        " Straight.");
}

bool Progressive::impl_apply(const PathPtr_t& path, PathPtr_t& proj) const {
  assert(path);
  bool success = false;
  PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST(PathVector, path);
  if (!pv) {
    if (!path->constraints() || !path->constraints()->configProjector()) {
      proj = path;
      success = true;
    } else {
      success = project(path, proj);
    }
  } else {
    PathVectorPtr_t res =
        PathVector::create(pv->outputSize(), pv->outputDerivativeSize());
    PathPtr_t part;
    success = true;
    for (size_t i = 0; i < pv->numberPaths(); i++) {
      if (!apply(pv->pathAtRank(i), part)) {
        // We add the path only if part is not NULL and:
        // - either its length is not zero,
        // - or it's not the first one.
        if (part && (part->length() > 0 || i == 0)) {
          res->appendPath(part);
        }
        success = false;
        break;
      }
      res->appendPath(part);
    }
    proj = res;
  }
  assert(proj);
  assert((proj->initial() - path->initial()).isZero());
  assert(!success || (proj->end() - path->end()).isZero());
  return success;
}

bool Progressive::project(const PathPtr_t& path, PathPtr_t& proj) const {
  ConstraintSetPtr_t constraints = path->constraints();
  if (!constraints) {
    proj = path;
    return true;
  }
  const ConfigProjectorPtr_t& cp = constraints->configProjector();
  core::interval_t timeRange = path->timeRange();
  const Configuration_t& q1 = path->initial();
  const Configuration_t& q2 = path->end();
  Configuration_t qtmp = q1;
  const size_t maxDichotomyTries = 10,
               maxPathSplit =
                   (size_t)(10 * (timeRange.second - timeRange.first) /
                            (double)step_);
  assert(constraints->isSatisfied(q1));
  if (!constraints->isSatisfied(q2)) return false;
  if (!cp || cp->dimension() == 0) {
    proj = path;
    return true;
  }

  bool pathIsFullyProjected = false;
  std::queue<PathPtr_t> paths;
  PathPtr_t toSplit = steer(q1, q2);
  Configuration_t qi(q1.size());
  value_type curStep, curLength, totalLength = 0;
  size_t c = 0;
  const value_type& K = hessianBound_;  // upper bound of Hessian
  if (withHessianBound_) cp->solver().oneStep(qtmp, lineSearch);
  value_type sigma = cp->sigma();

  value_type min = std::numeric_limits<value_type>::max(), max = 0;

  while (true) {
    const value_type threshold = (withHessianBound_ ? sigma / K : step_);
    const value_type thr_min = thresholdMin_;

    if (toSplit->length() < threshold) {
      paths.push(toSplit);
      assert(constraints->isSatisfied(toSplit->initial()));
      assert(constraints->isSatisfied(toSplit->end()));
      totalLength += toSplit->length();
      pathIsFullyProjected = true;
      break;
    }
    const Configuration_t& qb = toSplit->initial();
    curLength = std::numeric_limits<value_type>::max();
    size_t dicC = 0;

    curStep = threshold - Eigen::NumTraits<value_type>::epsilon();

    /* if (withHessianBound_) */ if (threshold < thr_min)
      break;
    // if (withHessianBound_) std::cout << threshold << std::endl;

    /// Find the good length.
    /// Here, it would be good to have an upper bound of the Hessian
    /// of the constraint.
    do {
      if (dicC >= maxDichotomyTries) break;
      toSplit->eval(qi, curStep);
      if (constraints->apply(qi)) curLength = d(qb, qi);
      curStep /= 2;
      dicC++;
    } while (curLength > threshold || curLength < 1e-3);
    if (dicC >= maxDichotomyTries || c > maxPathSplit) break;
    // if qi failed to be projected, stop.
    if (!constraints->isSatisfied(qi)) break;
    assert(curLength == d(qb, qi));
    assert(constraints->isSatisfied(qi));

    if (withHessianBound_) {
      /// Update sigma
      qtmp = qi;
      cp->solver().oneStep(qtmp, lineSearch);
      sigma = cp->sigma();
    }

    PathPtr_t part = steer(qb, qi);
    paths.push(part);
    assert(constraints->isSatisfied(part->initial()));
    assert(constraints->isSatisfied(part->end()));
    totalLength += part->length();
    min = std::min(min, part->length());
    max = std::max(max, part->length());
    toSplit = steer(qi, q2);
    c++;
  }
#if HPP_ENABLE_BENCHMARK
  hppBenchmark("Interpolated path (progressive): "
               << paths.size() << ", [ " << min << ", "
               << (paths.empty() ? 0 : totalLength / paths.size()) << ", "
               << max << "]");
#endif
  switch (paths.size()) {
    case 0:
      timeRange = path->timeRange();
      proj = path->extract(std::make_pair(timeRange.first, timeRange.first));
      return false;
      break;
    case 1:
      proj = paths.front()->copy(constraints);
      break;
    default:
      InterpolatedPathPtr_t p =
          InterpolatedPath::create(cp->robot(), q1, paths.back()->end(),
                                   totalLength, path->constraints());
      value_type t = paths.front()->length();
      qi = paths.front()->end();
      paths.pop();
      while (!paths.empty()) {
        assert((qi - paths.front()->initial()).isZero());
        qi = paths.front()->end();
        p->insert(t, paths.front()->initial());
        assert(t <= totalLength);
        t += paths.front()->length();
        paths.pop();
      }
      proj = p;
      break;
  }
  if (d(proj->initial(), path->initial()) != 0) {
    hppDout(error, "proj->initial () = " << proj->initial().transpose());
    hppDout(error, "path->initial ()       = " << path->initial().transpose());
  }
  assert(d(proj->initial(), path->initial()) == 0);
  if (pathIsFullyProjected && (d(proj->end(), path->end()) != 0)) {
    hppDout(error, "proj->end () = " << proj->end().transpose());
    hppDout(error, "path->end ()       = " << path->end().transpose());
    hppDout(error,
            "d (proj->end (), path->end ()) = " << d(proj->end(), path->end()));
    hppDout(error, "proj->end () - path->end () = "
                       << (proj->end() - path->end()).transpose());
  }
  assert(!pathIsFullyProjected || (d(proj->end(), path->end()) == 0));
  assert(proj->constraints()->isSatisfied(proj->end()));
#ifndef NDEBUG
  bool success;
  Configuration_t q = proj->eval(proj->timeRange().second, success);
  if (!success) {
    q = proj->eval(proj->timeRange().second, success);
  }
  assert(success);
  vector_t error;
  assert(proj->constraints()->isSatisfied(q, error));
#endif
  return pathIsFullyProjected;
}
}  // namespace pathProjector
}  // namespace core
}  // namespace hpp
