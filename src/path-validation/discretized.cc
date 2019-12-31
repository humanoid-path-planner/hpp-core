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

    bool Discretized::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
     PathValidationReportPtr_t& validationReport)
    {
        hppDout(notice,"path validation, reverse : "<<reverse);
      ValidationReportPtr_t configReport;
      assert (path);
      bool valid = true;
      if (reverse) {
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmax;
	value_type t = tmax;
	unsigned finished = 0;
        Configuration_t q (path->outputSize());
	while (finished < 2 && valid) {
          bool success = (*path) (q, t);
      if (!success || !ConfigValidations::validate (q, configReport)) {
	validationReport = CollisionPathValidationReportPtr_t
	  (new CollisionPathValidationReport (t, configReport));
	    valid = false;
	  } else {
	    lastValidTime = t;
	    t -= stepSize_;
	  }
	  if (t < tmin) {
	    t = tmin;
	    finished++;
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
          hppDout(notice,"path validation, else");
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmin;
	value_type t = tmin;
	unsigned finished = 0;
        Configuration_t q (path->outputSize());
	while (finished < 2 && valid) {
	  bool success = (*path) (q, t);
      if (!success || !ConfigValidations::validate (q, configReport)) {
	validationReport = CollisionPathValidationReportPtr_t
	  (new CollisionPathValidationReport (t, configReport));
	    valid = false;
	  } else {
	    lastValidTime = t;
	    t += stepSize_;
	  }
	  if (t > tmax) {
	    t = tmax;
	    finished ++;
	  }
	}
    hppDout(notice,"path validation, end. Valid : "<<valid<<" ; lastValidTime : "<<lastValidTime);
	if (valid) {
	  validPart = path;
	  return true;
	} else {
	  validPart = path->extract (std::make_pair (tmin, lastValidTime));
	  return false;
	}
      }
    }

    Discretized::Discretized (const value_type& stepSize) :
      stepSize_ (stepSize)
    {
    }

    } // namespace pathValidation
  } // namespace core
} // namespace hpp
