//
// Copyright (c) 2016 CNRS
// Authors: Pierre Fernbach
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

#include <sstream>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <hpp/core/torque-bound-validation.hh>

namespace hpp {
  namespace core {
    //typedef pinocchio::::JointConfiguration* JointConfigurationPtr_t;
    TorqueBoundValidationPtr_t TorqueBoundValidation::create
    (const DevicePtr_t& robot)
    {
      TorqueBoundValidation* ptr = new TorqueBoundValidation (robot);
      std::cout<<"Create torque validation"<<std::endl;
      return TorqueBoundValidationPtr_t (ptr);
    }
    


    // !!! specific implementation for double pendulum with hardcoded torque bounds and mass
    bool TorqueBoundValidation::validate
    (const Configuration_t& config, ValidationReportPtr_t& validationReport)
    {
      return true;
    }
    
    TorqueBoundValidation::TorqueBoundValidation (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
