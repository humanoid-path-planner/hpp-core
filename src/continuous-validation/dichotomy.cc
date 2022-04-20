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

#include <hpp/core/continuous-validation/dichotomy.hh>

#include <iterator>

#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-validation/intervals.hh"
#include "continuous-validation/helper.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {
      DichotomyPtr_t
      Dichotomy::create (const DevicePtr_t& robot, const value_type& tolerance)
      {
        Dichotomy* ptr = new Dichotomy (robot, tolerance);
        DichotomyPtr_t shPtr (ptr);
        ptr->init(shPtr);
        ptr->initialize();
        return shPtr;
      }

      Dichotomy::~Dichotomy ()
      {
      }

      bool Dichotomy::validateStraightPath
      (IntervalValidations_t& bpc, const PathConstPtr_t& path, bool reverse,
       PathPtr_t& validPart, PathValidationReportPtr_t& report)
      {
        if (reverse) return validateStraightPath<true> (bpc, path, validPart, report);
        else         return validateStraightPath<false>(bpc, path, validPart, report);
      }

      template <bool reverse>
      bool Dichotomy::validateStraightPath
      (IntervalValidations_t& bodyPairCollisions, const PathConstPtr_t& path,
       PathPtr_t& validPart, PathValidationReportPtr_t& report)
      {
        // start by validating end of path
        bool finished = false;
        bool valid = true;
        setPath(bodyPairCollisions, path, reverse);
        Intervals validSubset;
        const interval_t& tr (path->timeRange());
        value_type t = first(tr, reverse);
        validSubset.unionInterval (std::make_pair (t,t));
        Configuration_t q (path->outputSize ());
        value_type t0, t1, tmin, tmax;
        while (!finished) {
          bool success = (*path) (q, t);
          PathValidationReportPtr_t pathReport;
          interval_t interval;
          if (!success) {
            report = PathValidationReportPtr_t (new PathValidationReport (t,
                  ValidationReportPtr_t(new ProjectionError())));
            valid = false;
          } else if (!validateConfiguration (bodyPairCollisions, q, t, interval, pathReport)) {
            report = pathReport;
            valid = false;
          } else {
            validSubset.unionInterval (interval);
          }
          finished = (!valid || validSubset.contains (tr));
          // Compute next parameter to check as middle of first non tested
          // interval
          t0 = second (begin(validSubset.list(), reverse), reverse);
          t1 = second (tr                                , reverse);
          if (validSubset.list().size() > 1)
            t1 = first (Nth(validSubset.list(), 1, reverse), reverse);
          t = .5* (t0 + t1);
        }
        if (!valid) {
          assert ( begin(validSubset.list (), reverse).first <=
                   first(tr                 , reverse));
          assert ( begin(validSubset.list (), reverse).second >=
                   first(tr                 , reverse));
          if (reverse) {
            tmin = validSubset.list ().rbegin ()->first;
            tmax = tr.second;
          } else {
            tmin = tr.first;
            tmax = validSubset.list ().begin ()->second;
          }
          validPart = path->extract (tmin, tmax);
          return false;
        } else {
          validPart = path->copy();
          return true;
        }
        return false;
      }

      void Dichotomy::init(const DichotomyWkPtr_t weak)
      {
        ContinuousValidation::init (weak);
        weak_ = weak;
      }

      Dichotomy::Dichotomy (const DevicePtr_t& robot, const value_type& tolerance):
	      ContinuousValidation (robot, tolerance), weak_()
      {
        // Tolerance should be equal to 0, otherwise end of valid
        // sub-path might be in collision.
        if (tolerance != 0) {
          throw std::runtime_error ("Dichotomy path validation method does not"
                  "support penetration.");
        }
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
