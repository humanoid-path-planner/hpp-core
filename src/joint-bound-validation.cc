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
