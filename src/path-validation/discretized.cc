//
// Copyright (c) 2015 CNRS
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
    } // namespace pathValidation
  } // namespace core
} // namespace hpp
