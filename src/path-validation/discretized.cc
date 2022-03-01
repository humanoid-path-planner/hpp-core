//
// Copyright (c) 2015 CNRS
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

#include <hpp/pinocchio/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/util/debug.hh>

namespace hpp {
  namespace core {
    namespace pathValidation {

    DiscretizedPtr_t
    Discretized::create (const value_type& stepSize)
    {
      Discretized* ptr = new Discretized(stepSize);
      return DiscretizedPtr_t (ptr);
    }

    DiscretizedPtr_t
    Discretized::create (const value_type& stepSize,
        std::initializer_list<ConfigValidationPtr_t> validations)
    {
      Discretized* ptr = new Discretized(stepSize, validations);
      return DiscretizedPtr_t (ptr);
    }

    bool Discretized::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
     PathValidationReportPtr_t& validationReport)
    {
        hppDout(notice,"path validation, reverse : "<<reverse);
      ValidationReportPtr_t configReport;
      assert (path);
      bool valid = true;
      const value_type tmin = path->timeRange ().first;
      const value_type tmax = path->timeRange ().second;
      unsigned finished = 0;
      Configuration_t q (path->outputSize());
      value_type T1, step, lastValidTime;
      if (reverse) {
        lastValidTime = tmax;
        T1 = tmin;
        step = -stepSize_;
      } else {
        lastValidTime = tmin;
        T1 = tmax;
        step = stepSize_;
      }

      value_type t = lastValidTime;
      while (finished < 2 && valid) {
        bool success = (*path) (q, t);
        if (!success) {
          validationReport = PathValidationReportPtr_t (
              new PathValidationReport (t,
                ValidationReportPtr_t(new ProjectionError()))
              );
          valid = false;
        } else if (!ConfigValidations::validate (q, configReport)) {
          validationReport = CollisionPathValidationReportPtr_t
            (new CollisionPathValidationReport (t, configReport));
          valid = false;
        } else {
          lastValidTime = t;
          t += step;
        }
        if (   ( reverse && t < T1)
            || (!reverse && t > T1)) {
          t = T1;
          finished++;
        }
      }
      if (valid) {
        validPart = path;
        return true;
      } else {
        if (reverse)
          validPart = path->extract (lastValidTime, tmax);
        else
	  validPart = path->extract (tmin, lastValidTime);
        return false;
      }
    }

    bool Discretized::validate(ConfigurationIn_t q,
                               ValidationReportPtr_t& report)
    {
      return ConfigValidations::validate(q, report);
    }

    } // namespace pathValidation
  } // namespace core
} // namespace hpp
