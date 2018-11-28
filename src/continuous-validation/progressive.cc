//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/core/continuous-validation/progressive.hh>

#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/continuous-validation/initializer.hh>

#include "continuous-validation/intervals.hh"
#include "continuous-validation/helper.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {
      namespace {
        typedef std::pair<pinocchio::JointIndex, pinocchio::JointIndex> JointIndexPair_t;

        struct JointIndexPairCompare_t {
          bool operator() (const JointIndexPair_t& p0, const JointIndexPair_t& p1) const
          {
            if (p0.first < p1.first) return true;
            if (p0.first > p1.first) return false;
            return (p0.second < p1.second);
          }
        };

        typedef std::set<JointIndexPair_t, JointIndexPairCompare_t> JointIndexPairSet_t;
      }

      ProgressivePtr_t Progressive::create (const DevicePtr_t& robot,
					    const value_type& tolerance)
      {
        Progressive* ptr = new Progressive (robot, tolerance);
        ProgressivePtr_t shPtr (ptr);
        ptr->init(shPtr);
        ptr->initializer_->initialize();
        return shPtr;
      }

      bool Progressive::validateStraightPath
      (BodyPairCollisions_t& bpc, const PathPtr_t& path,
       bool reverse, PathPtr_t& validPart, PathValidationReportPtr_t& report)
      {
        if (reverse) return validateStraightPath<true> (bpc, path, validPart, report);
        else         return validateStraightPath<false>(bpc, path, validPart, report);
      }

      template <bool reverse>
      bool Progressive::validateStraightPath
      (BodyPairCollisions_t& bodyPairCollisions, const PathPtr_t& path,
       PathPtr_t& validPart, PathValidationReportPtr_t& report)
      {
        // for each IntervalValidation
        //   - set path,
        //   - compute valid interval at start (end if reverse)
        bool valid = true;
        const interval_t& tr (path->timeRange());
        Configuration_t q (path->outputSize ());
        PathValidationReportPtr_t pathReport;
        interval_t interval;

        setPath(bodyPairCollisions, path, reverse);

        const value_type tmin = tr.first;
        const value_type tmax = tr.second;
        value_type lastValidTime = first (tr, reverse);
        value_type t             = lastValidTime;
        unsigned finished = 0;
        // If the interval is of length 0, there is only one configuration to
        // validate.
        if (reverse ? t <= tmin : t >= tmax) finished++;
        while (finished < 2 && valid) {
          bool success = (*path) (q, t);
          value_type tprev = t;
          if (!success ||
              !validateConfiguration (bodyPairCollisions, q, t, interval, pathReport)) {
            report = pathReport;
            valid = false;
          } else {
            t = second (interval, reverse);
            lastValidTime = tprev;
          }
          if (reverse ? t <= tmin : t >= tmax) {
            t = reverse ? tmin : tmax;
            finished ++;
          }
        }
        if (valid) {
          validPart = path;
          return true;
        } else {
          validPart = reverse ? 
              path->extract (std::make_pair (lastValidTime, tmax))
            : path->extract (std::make_pair (tmin, lastValidTime));
          return false;
        }
      }

      Progressive::~Progressive ()
      {
      }

      void Progressive::init(const ProgressiveWkPtr_t weak)
      {
        ContinuousValidation::init (weak);
        weak_ = weak;
      }

      Progressive::Progressive (const DevicePtr_t& robot, const value_type& tolerance):
	      ContinuousValidation (robot, tolerance), weak_()
      {
        if (tolerance <= 0) {
          throw std::runtime_error
            ("tolerance should be positive for"
            " progressive continuous validation.");
        }
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
