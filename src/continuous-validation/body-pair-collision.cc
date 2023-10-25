// Copyright (c) 2014,2015,2016,2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#define HPP_DEBUG 1

#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>

#include <hpp/core/continuous-validation/body-pair-collision.hh>
#include <hpp/core/path.hh>
#include <hpp/core/straight-path.hh>  // To enable dynamic casting (needs inheritance).
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <limits>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace hpp {
namespace core {
namespace continuousValidation {
using ::pinocchio::toFclTransform3f;

bool BodyPairCollision::validateConfiguration(
    const value_type& t, interval_t& interval, ValidationReportPtr_t& report,
    const pinocchio::DeviceData& data) {
  namespace icl = boost::icl;
  using std::numeric_limits;

  if (valid_) {
    interval = path_->timeRange();
    assert(interval.second > interval.first);
    return true;
  }
  continuous_interval iclInterval(interval.first, interval.second,
                                  icl::interval_bounds::closed());
  if (icl::contains(validInterval_, iclInterval)) {
    // TODO interval could probably be enlarge using validInterval_
    // interval = validInterval_;
    assert(interval.second > interval.first);
    return true;
  }

  value_type distanceLowerBound;
  if (!computeDistanceLowerBound(distanceLowerBound, report, data)) {
    return false;
  }

  value_type halfLengthDist, halfLengthTol;
  /// \todo A finer bound could be computed when path is an
  ///       InterpolatedPath using the maximal velocity on each
  ///       subinterval
  if (distanceLowerBound == numeric_limits<value_type>::infinity()) {
    halfLengthDist = numeric_limits<value_type>::infinity();
    halfLengthTol = 0;
  } else {
    value_type Vm;
    halfLengthDist = collisionFreeInterval(t, distanceLowerBound, Vm);
    if (Vm != 0) {
      halfLengthTol = 2 * tolerance_ / Vm;
    } else {
      halfLengthTol = numeric_limits<value_type>::infinity();
    }
  }
  assert(!std::isnan(halfLengthDist));
  assert(!std::isnan(halfLengthTol));
  interval.first = t - (halfLengthDist + halfLengthTol);
  interval.second = t + (halfLengthDist + halfLengthTol);
  validInterval_.insert(continuous_interval(interval.first, interval.second,
                                            icl::interval_bounds::closed()));
  // Check if the whole path is valid.
  iclInterval =
      continuous_interval(path_->timeRange().first, path_->timeRange().second,
                          icl::interval_bounds::closed());
  if (icl::contains(validInterval_, iclInterval)) valid_ = true;
  assert(interval.second > interval.first || path()->length() == 0);
  return true;
}

void BodyPairCollision::setupPath() {
  if (HPP_DYNAMIC_PTR_CAST(StraightPath, path_))
    refine_ = false;
  else
    refine_ = true;
  Vb_ = vector_t(path_->outputDerivativeSize());
  value_type t0 = path_->timeRange().first;
  value_type t1 = path_->timeRange().second;
  assert(t1 >= t0);
  if (t1 - t0 == 0) {
    maximalVelocity_ = std::numeric_limits<value_type>::infinity();
    refine_ = false;
  } else {
    path_->velocityBound(Vb_, t0, t1);
    maximalVelocity_ = computeMaximalVelocity(Vb_);
  }
}

value_type BodyPairCollision::collisionFreeInterval(
    const value_type& t, const value_type& distanceLowerBound,
    value_type& maxVelocity) const {
  constexpr int Nrefine = 2;
  value_type T[1 + 2 * Nrefine], Vm[1 + 2 * Nrefine];
  value_type tm, tM;
  Vm[0] = maxVelocity = maximalVelocity_;
  T[0] = distanceLowerBound / maxVelocity;
  if (!refine_) {
    if (T[0] < 1e-3) {
      hppDout(notice,
              "Small interval without refine: "
              "maxVelocity = "
                  << maxVelocity << " / T = " << T[0]
                  << " / d = " << distanceLowerBound);
    }
    return T[0];
  } else {
    tm = t - T[0];
    tM = t + T[0];
    bool leftIsValid = (tm < path_->timeRange().first);
    bool rightIsValid = (tM > path_->timeRange().second);
    if (leftIsValid && rightIsValid) return T[0];

    // Refinement step
    // Make Nrefine times the following actions
    //
    //  1. Recompute the maximal velocity on interval T[2*i+0] -> Vm[2*i+1]
    //  2. T[2*i+1] <- distanceLowerBound / Vm[2*i+1]
    //  3. Recompute the maximal velocity on interval T[2*i+1] -> Vm[2*i+2]
    //  4. T[2*i+2] <- distanceLowerBound / Vm[2*i+1]
    //
    //  The following inequalities hold:
    //    - T [2*i+1] >=  T [2*i+0],
    //    - Vm[2*i+1] <=  Vm[2*i],
    //    - T [2*i+2] <=  T [2*i+1],
    //    - Vm[2*i+2] >=  Vm[2*i+1]
    // since
    //    - the maximal velocity over an interval is increasing with the
    //      size of the interval and
    //    - distanceLowerBound / Vm is decreasing when Vm increases.
    //
    // Note that steps 3 and 4 are compacted into one loop in the lines
    // below. Performing those steps an even number of times ensures that
    // the maximal velocity used to compute the result interval T[2*i+2]
    // is evaluated on an superset of T[2*i+2].
    for (int i = 0; i < 2 * Nrefine; ++i) {
      tm = t - (leftIsValid ? 0 : T[i]);
      tM = t + (rightIsValid ? 0 : T[i]);
      path_->velocityBound(Vb_, tm, tM);
      Vm[i + 1] = computeMaximalVelocity(Vb_);
      T[i + 1] = distanceLowerBound / Vm[i + 1];
      if (i % 2 == 1) {
        assert(T[i + 1] >= T[i]);
        assert(Vm[i + 1] <= Vm[i]);
        assert(T[i + 2] <= T[i + 1]);
        assert(Vm[i + 2] >= Vm[i + 1]);
      }
    }
    constexpr int k = 2 * Nrefine;
    maxVelocity = Vm[k];
    if (T[k] < 1e-3) {
      hppDout(notice,
              "Small interval with refine: "
              "maxVelocity = "
                  << maxVelocity << " / T = " << T[k]
                  << " / d = " << distanceLowerBound);
    }
    return T[k];
  }
}

bool BodyPairCollision::computeDistanceLowerBound(
    value_type& distanceLowerBound, ValidationReportPtr_t& report,
    const pinocchio::DeviceData& data) {
  using std::numeric_limits;
  distanceLowerBound = numeric_limits<value_type>::infinity();
  const CollisionPairs_t& prs(pairs());
  CollisionRequests_t& rqsts(requests());
  assert(rqsts.size() == prs.size());
  for (std::size_t i = 0; i < prs.size(); ++i) {
    assert(rqsts[i].enable_distance_lower_bound == true);
    fcl::CollisionResult result;
    prs[i].collide(data, rqsts[i], result);
    // Get result
    if (result.isCollision()) {
      setReport(report, result, prs[i]);
      return false;
    }
    if (result.distance_lower_bound < distanceLowerBound) {
      distanceLowerBound = result.distance_lower_bound;
      assert(distanceLowerBound > 0);
    }
  }
  return true;
}
}  // namespace continuousValidation
}  // namespace core
}  // namespace hpp
