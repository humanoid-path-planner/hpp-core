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
#include <hpp/model/configuration.hh>
#include <hpp/core/collision-validation.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    typedef model::JointConfiguration* JointConfigurationPtr_t;
    CollisionValidationPtr_t CollisionValidation::create
    (const DevicePtr_t& robot)
    {
      CollisionValidation* ptr = new CollisionValidation (robot);
      return CollisionValidationPtr_t (ptr);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					bool report)
    {
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics ();
      if (robot_->collisionTest ()) {
	if (report) {
	  std::ostringstream oss ("Configuration in collision: ");
	  oss << displayConfig (config);
	  throw std::runtime_error (oss.str ());
	} else {
	  return false;
	}
      }
      return true;
    }

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
