//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/core/continuous-validation/progressive.hh>

#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

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
        ptr->initialize();
        return shPtr;
      }

      bool Progressive::validateStraightPath
      (IntervalValidations_t& bpc, const PathConstPtr_t& path,
       bool reverse, PathPtr_t& validPart, PathValidationReportPtr_t& report)
      {
        if (reverse) return validateStraightPath<true> (bpc, path, validPart, report);
        else         return validateStraightPath<false>(bpc, path, validPart, report);
      }

      template <bool reverse>
      bool Progressive::validateStraightPath
      (IntervalValidations_t& bodyPairCollisions, const PathConstPtr_t& path,
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
          if (!success) {
            report = PathValidationReportPtr_t (new PathValidationReport (t,
                  ValidationReportPtr_t(new ProjectionError())));
            valid = false;
          } else if (!validateConfiguration (bodyPairCollisions, q, t, interval, pathReport)) {
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
          validPart = path->copy();
          return true;
        } else {
          validPart = reverse ? 
              path->extract (lastValidTime, tmax)
            : path->extract (tmin, lastValidTime);
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
