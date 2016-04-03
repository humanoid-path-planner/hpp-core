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

#include <hpp/model/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/core/discretized-path-validation.hh>

namespace hpp {
  namespace core {

    DiscretizedPathValidationPtr_t
    DiscretizedPathValidation::create (const DevicePtr_t& robot,
				       const value_type& stepSize)
    {
      DiscretizedPathValidation* ptr =
	new DiscretizedPathValidation(robot, stepSize);
      return DiscretizedPathValidationPtr_t (ptr);
    }

    void DiscretizedPathValidation::add
    (const ConfigValidationPtr_t& configValidation)
    {
      configValidations_->add (configValidation);
    }

    void DiscretizedPathValidation::addObstacle
    (const CollisionObjectPtr_t& object)
    {
      configValidations_->addObstacle (object);
    }

    bool DiscretizedPathValidation::validate
    (const PathPtr_t&, bool, PathPtr_t&)
    {
      throw std::runtime_error
	("Deprecated DiscretizedPathValidation::validate: Not implemented");
    }

    bool DiscretizedPathValidation::validate
    (const PathPtr_t&, bool, PathPtr_t&, ValidationReport&)
    {
      throw std::runtime_error
	("Deprecated DiscretizedPathValidation::validate: Not implemented");
    }

    bool DiscretizedPathValidation::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
     PathValidationReportPtr_t& validationReport)
    {
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
      if (!success || !configValidations_->validate (q, configReport)) {
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
	value_type tmin = path->timeRange ().first;
	value_type tmax = path->timeRange ().second;
	value_type lastValidTime = tmin;
	value_type t = tmin;
	unsigned finished = 0;
        Configuration_t q (path->outputSize());
	while (finished < 2 && valid) {
	  bool success = (*path) (q, t);
      if (!success || !configValidations_->validate (q, configReport)) {
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
	if (valid) {
	  validPart = path;
	  return true;
	} else {
	  validPart = path->extract (std::make_pair (tmin, lastValidTime));
	  return false;
	}
      }
    }

    void DiscretizedPathValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
      configValidations_->removeObstacleFromJoint (joint, obstacle);
    }

    size_type DiscretizedPathValidation::filterCollisionPairs (
        const ConstraintSetPtr_t& constraints)
    {
      return configValidations_->filterCollisionPairs (constraints);
    }

    DiscretizedPathValidation::DiscretizedPathValidation
    (const DevicePtr_t& robot, const value_type& stepSize) :
      stepSize_ (stepSize), robot_ (robot),
      configValidations_ (ConfigValidations::create ())
    {
    }
  } // namespace core
} // namespace hpp
