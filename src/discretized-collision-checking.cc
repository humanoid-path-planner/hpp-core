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

#include <hpp/model/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/core/discretized-collision-checking.hh>

namespace hpp {
  namespace core {

    DiscretizedCollisionCheckingPtr_t
    DiscretizedCollisionChecking::createWithValidation (const DevicePtr_t& robot,
					  const value_type& stepSize,
				    const PathValidationReport& defaultValidationReport,
				    const ConfigValidationPtr_t& configValidation)
    {
      DiscretizedCollisionChecking* ptr =
	new DiscretizedCollisionChecking(robot, stepSize, defaultValidationReport,
		configValidation);
      return DiscretizedCollisionCheckingPtr_t (ptr);
    }

    DiscretizedCollisionCheckingPtr_t
    DiscretizedCollisionChecking::create (const DevicePtr_t& robot,
					  const value_type& stepSize)
    {
			CollisionPathValidationReport unusedReport;
      DiscretizedCollisionChecking* ptr =
	new DiscretizedCollisionChecking(robot, stepSize, unusedReport
		, CollisionValidation::create (robot));
      return DiscretizedCollisionCheckingPtr_t (ptr);
    }

    void DiscretizedCollisionChecking::addObstacle
    (const CollisionObjectPtr_t& object)
    {
      configValidation_->addObstacle (object);
    }

    bool DiscretizedCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      return validate (path, reverse, validPart, unusedReport_);
    }

    bool DiscretizedCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
     ValidationReport& validationReport)
    {
      HPP_STATIC_CAST_REF_CHECK (PathValidationReport,
				 validationReport);
      PathValidationReport& report =
      static_cast <PathValidationReport&> (validationReport);
      assert (path);
      bool valid = true;
      if (reverse) {
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmax;
	value_type t = tmax;
	unsigned finished = 0;
	while (finished < 2 && valid) {
	  Configuration_t q = (*path) (t);
      if (!configValidation_->validate (q, *report.configurationReport, false)) {
	    report.parameter = t;	    
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
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmin;
	value_type t = tmin;
	unsigned finished = 0;
	while (finished < 2 && valid) {
	  Configuration_t q = (*path) (t);
      if (!configValidation_->validate (q, *report.configurationReport, false)) {
	    report.parameter = t;	    
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
	if (valid) {
	  validPart = path;
	  return true;
	} else {
	  validPart = path->extract (std::make_pair (tmin, lastValidTime));
	  return false;
	}
      }
    }

    DiscretizedCollisionChecking::DiscretizedCollisionChecking
    (const DevicePtr_t& robot, const value_type& stepSize,
				    const PathValidationReport& defaultValidationReport,
				    const ConfigValidationPtr_t& configValidation) :
      PathValidation (), robot_ (robot),
      configValidation_ (configValidation),
      stepSize_ (stepSize),
      unusedReport_(defaultValidationReport)
    {
    }

    void DiscretizedCollisionChecking::removeObstacleFromJoint (const JointPtr_t& joint,
        const CollisionObjectPtr_t& obstacle)
    {
      assert (configValidation_);
      configValidation_->removeObstacleFromJoint (joint, obstacle);
    }
  } // namespace core
} // namespace hpp
