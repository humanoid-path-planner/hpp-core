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
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/continuous-validation/initializer.hh>

#include "continuous-validation/intervals.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {
      namespace {
        typedef std::pair<se3::JointIndex, se3::JointIndex> JointIndexPair_t;

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
      (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
       PathValidationReportPtr_t& report)
      {
        // for each IntervalValidation
        //   - set path,
        //   - compute valid interval at start (end if reverse)
        bool valid = true;
        setPath(path, reverse);
        if (reverse) {
          value_type tmin = path->timeRange ().first;
          value_type tmax = path->timeRange ().second;
          value_type lastValidTime = tmax;
          value_type t = tmax;
          unsigned finished = 0;
          Configuration_t q (path->outputSize ());
          while (finished < 2 && valid) {
            bool success = (*path) (q, t);
            value_type tprev = t;
            PathValidationReportPtr_t pathReport;
            interval_t interval;
            if (!success || !validateConfiguration (q, t, interval,
                      pathReport)) {
              report = pathReport;
              valid = false;
            } else {
              t = interval.first;
              lastValidTime = tprev;
            }
            if (t <= tmin) {
              t = tmin;
              finished ++;
            }
          }
          if (valid) {
            validPart = path;
            return true;
          } else {
            validPart = path->extract (std::make_pair (lastValidTime, tmax));
            return false;
          }
        } else {
          value_type tmin = path->timeRange ().first;
          value_type tmax = path->timeRange ().second;
          value_type lastValidTime = tmin;
          value_type t = tmin;
          unsigned finished = 0;
          Configuration_t q (path->outputSize ());
          if (t >= tmax) finished++;
          while (finished < 2 && valid) {
            bool success = (*path) (q, t);
            value_type tprev = t;
            PathValidationReportPtr_t pathReport;
            interval_t interval;
            if (!success || !validateConfiguration (q, t, interval,
                      pathReport)) {
              report = pathReport;
              valid = false;
            } else {
              t = interval.second;
              lastValidTime = tprev;
            }
            if (t >= tmax) {
              t = tmax;
              finished ++;
            }
          }
          if (valid) {
            validPart = path;
            return true;
          } else {
            validPart = path->extract (std::make_pair (tmin, lastValidTime));
            return false;
          }
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
