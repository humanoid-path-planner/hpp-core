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


#include <hpp/core/config-validations.hh>
#include <hpp/core/validation-report.hh>

namespace hpp {
  namespace core {
    ConfigValidationsPtr_t ConfigValidations::create ()
    {
      ConfigValidations* ptr = new ConfigValidations;
      return ConfigValidationsPtr_t (ptr);
    }

    bool ConfigValidations::validate (const Configuration_t& config,
				      ValidationReportPtr_t& validationReport)
    {
      for (std::vector <ConfigValidationPtr_t>::iterator
	     it = validations_.begin (); it != validations_.end (); ++it) {
	if ((*it)->validate (config, validationReport)
	    == false) {
	  return false;
	}
      }
      return true;
    }

    void ConfigValidations::add (const ConfigValidationPtr_t& configValidation)
    {
      validations_.push_back (configValidation);
    }

    void ConfigValidations::addObstacle (const CollisionObjectPtr_t& object)
    {
      for (std::vector <ConfigValidationPtr_t>::iterator itVal =
	     validations_.begin (); itVal != validations_.end (); ++itVal) {
	(*itVal)->addObstacle (object);
      }
    }

    void ConfigValidations::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
      for (std::vector <ConfigValidationPtr_t>::iterator itVal =
	     validations_.begin (); itVal != validations_.end (); ++itVal) {
	(*itVal)->removeObstacleFromJoint (joint, obstacle);
      }
    }

    void ConfigValidations::filterCollisionPairs (const RelativeMotion::matrix_type& matrix)
    {
      for (std::vector <ConfigValidationPtr_t>::iterator itVal =
	     validations_.begin (); itVal != validations_.end (); ++itVal) {
	(*itVal)->filterCollisionPairs (matrix);
      }
    }

    ConfigValidations::ConfigValidations () : validations_ ()
    {
    }

  } // namespace core
} // namespace hpp
