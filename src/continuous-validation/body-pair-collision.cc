// Copyright (c) 2014,2015,2016,2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#define HPP_DEBUG 1

#include <hpp/core/continuous-validation/body-pair-collision.hh>

#include <limits>

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision.h>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>

#include <hpp/core/path.hh>
#include <hpp/core/straight-path.hh> // To enable dynamic casting (needs inheritance).

namespace hpp {
  namespace core {
    namespace continuousValidation {

      bool BodyPairCollision::validateConfiguration (const value_type& t, interval_t& interval,
                CollisionValidationReportPtr_t& report,
                pinocchio::DeviceData& data)
      {
        namespace icl = boost::icl;
        using std::numeric_limits;

        if (valid_) {
          interval = path_->timeRange ();
          assert (interval.second > interval.first);
          return true;
        }
        continuous_interval iclInterval (interval.first, interval.second,
          icl::interval_bounds::closed());
        if (icl::contains (validInterval_, iclInterval))
        {
          // TODO interval could probably be enlarge using validInterval_
          // interval = validInterval_;
          assert (interval.second > interval.first);
          return true;
        }

        value_type distanceLowerBound;
        if (!computeDistanceLowerBound(distanceLowerBound, report, data))
        {
          return false;
        }

        value_type halfLengthDist, halfLengthTol;
        /// \todo A finer bound could be computed when path is an
        ///       InterpolatedPath using the maximal velocity on each
        ///       subinterval
        if (distanceLowerBound == numeric_limits <value_type>::infinity ()) {
          halfLengthDist = numeric_limits <value_type>::infinity ();
          halfLengthTol = 0;
        } else {
          value_type Vm;
          halfLengthDist = collisionFreeInterval(t, distanceLowerBound, Vm);
          if (Vm != 0) {
            halfLengthTol = 2*tolerance_/Vm;
          } else {
            halfLengthTol = numeric_limits <value_type>::infinity ();
          }
        }
        assert (!std::isnan (halfLengthDist));
        assert (!std::isnan (halfLengthTol));
        interval.first = t - (halfLengthDist + halfLengthTol);
        interval.second = t + (halfLengthDist + halfLengthTol);
        validInterval_.insert (continuous_interval(interval.first,
          interval.second, icl::interval_bounds::closed()));
        // Check if the whole path is valid.
        iclInterval = continuous_interval (path_->timeRange ().first,
          path_->timeRange ().second, icl::interval_bounds::closed());
        if (icl::contains (validInterval_, iclInterval))
          valid_ = true;
        assert (interval.second > interval.first);
        return true;
      }

      void BodyPairCollision::setupPath()
      {
        if (HPP_DYNAMIC_PTR_CAST(StraightPath, path_)) refine_ = false;
        Vb_ = vector_t (path_->outputDerivativeSize());
        value_type t0 = path_->timeRange ().first;
        value_type t1 = path_->timeRange ().second;
        assert (t1 >= t0);
        if (t1 - t0 == 0) {
          maximalVelocity_ = std::numeric_limits<value_type>::infinity();
          refine_ = false;
        } else {
          path_->velocityBound (Vb_, t0, t1);
          maximalVelocity_ = computeMaximalVelocity (Vb_);
        }
      }

      value_type BodyPairCollision::collisionFreeInterval(const value_type &t,
                                      const value_type &distanceLowerBound,
                                      value_type &maxVelocity) const
      {
        value_type T[3], Vm[2];
        value_type tm, tM;
        maxVelocity = maximalVelocity_;
        T[0] = distanceLowerBound / maxVelocity;
        if (!refine_)
          return T[0];
        else
        {
          tm = t - T[0];
          tM = t + T[0];
          bool  leftIsValid = (tm < path_->timeRange().first );
          bool rightIsValid = (tM > path_->timeRange().second);
          if (leftIsValid && rightIsValid)
            return T[0];

          for (int i = 0; i < 2; ++i)
          {
            tm = t - (leftIsValid ? 0 : T[i]);
            tM = t + (rightIsValid ? 0 : T[i]);
            path_->velocityBound(Vb_, tm, tM);
            Vm[i] = computeMaximalVelocity(Vb_);
            T[i + 1] = distanceLowerBound / Vm[i];
          }
          assert(maximalVelocity_ >= Vm[1] && Vm[1] >= Vm[0] && T[1] >= T[2] && T[2] >= T[0]);
          // hppDout (info, "Refine changed the interval length of " << T[0] / T[2] << ", from " << T[0] << " to " << T[2]);
          maxVelocity = Vm[1];
          return T[2];
        }
      }

      bool BodyPairCollision::computeDistanceLowerBound(value_type &distanceLowerBound,
        CollisionValidationReportPtr_t& report,
        pinocchio::DeviceData& data) const
      {
        using std::numeric_limits;
        distanceLowerBound = numeric_limits <value_type>::infinity ();
        static const fcl::CollisionRequest request (fcl::DISTANCE_LOWER_BOUND, 1);
        assert (request.enable_distance_lower_bound == true);
        const CollisionPairs_t& prs (pairs());
        for (CollisionPairs_t::const_iterator _pair = prs.begin();
            _pair != prs.end(); ++_pair) {
          pinocchio::FclConstCollisionObjectPtr_t object_a = _pair->first ->fcl (data);
          pinocchio::FclConstCollisionObjectPtr_t object_b = _pair->second->fcl (data);
          fcl::CollisionResult result;
          fcl::collide (object_a, object_b, request, result);
          // Get result
          if (result.isCollision ()) {
            setReport(report, result, *_pair);
            return false;
          }
          if (result.distance_lower_bound < distanceLowerBound) {
            distanceLowerBound = result.distance_lower_bound;
            assert (distanceLowerBound > 0);
          }
        }
        return true;
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
