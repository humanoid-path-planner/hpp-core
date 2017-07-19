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

#include <hpp/core/joint-bound-validation.hh>

#include <sstream>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace core {
    typedef pinocchio::JointConfiguration* JointConfigurationPtr_t;
    JointBoundValidationPtr_t JointBoundValidation::create
    (const DevicePtr_t& robot)
    {
      JointBoundValidation* ptr = new JointBoundValidation (robot);
      return JointBoundValidationPtr_t (ptr);
    }

    bool JointBoundValidation::validate
    (const Configuration_t& config, ValidationReportPtr_t& validationReport)
    {
      const pinocchio::Model& model = robot_->model();
      // Check whether all config param are within boundaries.
      for (std::size_t i = 0; i < (std::size_t)model.nq; ++i) {
        if ( (model.upperPositionLimit[i] < config[i])
          || (model.lowerPositionLimit[i] > config[i])) {
          /// Find the joint at rank i
          JointPtr_t joint = robot_->getJointAtConfigRank(i);
          assert (i >= (std::size_t)joint->rankInConfiguration());
          const std::size_t j = i - joint->rankInConfiguration();

          JointBoundValidationReportPtr_t report(new JointBoundValidationReport
              (joint, j, model.lowerPositionLimit[i],
               model.upperPositionLimit[i], config[i]));
          validationReport = report;
          return false;
        }
      }
      const pinocchio::ExtraConfigSpace& ecs = robot_->extraConfigSpace();
      // Check the extra config space
      // FIXME This was introduced at the same time as the integration of Pinocchio
      size_type index = robot_->model().nq;
      for (size_type i=0; i < ecs.dimension(); ++i) {
        value_type lower = ecs.lower (i);
        value_type upper = ecs.upper (i);
        value_type value = config [index + i];
        if (value < lower || upper < value) {
          JointBoundValidationReportPtr_t report(new JointBoundValidationReport (JointPtr_t(), i, lower, upper, value));
          validationReport = report;
          return false;
        }
      }
      return true;
    }

    JointBoundValidation::JointBoundValidation (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
