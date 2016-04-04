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

#include <sstream>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/joint-bound-validation.hh>

namespace hpp {
  namespace core {
    typedef model::JointConfiguration* JointConfigurationPtr_t;
    JointBoundValidationPtr_t JointBoundValidation::create
    (const DevicePtr_t& robot)
    {
      JointBoundValidation* ptr = new JointBoundValidation (robot);
      return JointBoundValidationPtr_t (ptr);
    }

    bool JointBoundValidation::validate
    (const Configuration_t& config, ValidationReportPtr_t& validationReport)
    {
      const JointVector_t jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); ++itJoint) {
	size_type index = (*itJoint)->rankInConfiguration ();
	JointConfigurationPtr_t jc = (*itJoint)->configuration ();
	for (size_type i=0; i < (*itJoint)->configSize (); ++i) {
	  if (jc->isBounded (i)) {
	    value_type lower = jc->lowerBound (i);
	    value_type upper = jc->upperBound (i);
	    value_type value = config [index + i];
	    if (value < lower || upper < value) {
	      JointBoundValidationReportPtr_t report
		(new JointBoundValidationReport (*itJoint, i, lower, upper,
						 value));
	      validationReport = report;
	      return false;
	    }
	  }
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
