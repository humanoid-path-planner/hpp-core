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
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/torque-bound-validation.hh>

namespace hpp {
  namespace core {
    typedef model::JointConfiguration* JointConfigurationPtr_t;
    TorqueBoundValidationPtr_t TorqueBoundValidation::create
    (const DevicePtr_t& robot)
    {
      TorqueBoundValidation* ptr = new TorqueBoundValidation (robot);
      return TorqueBoundValidationPtr_t (ptr);
    }
    


    // !!! specific implementation for double pendulum with hardcoded torque bounds and mass
    bool TorqueBoundValidation::validate
    (const Configuration_t& config, ValidationReportPtr_t& validationReport)
    {
      const JointVector_t jv = robot_->getJointVector ();
      /*for (JointVector_t::const_iterator itJoint = jv.begin ();
           itJoint != jv.end (); ++itJoint) {
        size_type index = (*itJoint)->rankInConfiguration ();
        JointConfigurationPtr_t jc = (*itJoint)->configuration ();
        for (size_type i=0; i < (*itJoint)->configSize (); ++i) {
          if (jc->isBounded (i)) { // TODO change condition, need to add method in class Joint
            value_type value,bound;            
            bound = 15;
            // TODO : compute value = torque of this joint


            if (value > bound) {
              TorqueBoundValidationReportPtr_t report
                  (new TorqueBoundValidationReport (*itJoint, i, bound,value));
              validationReport = report;
              return false;
            }
          }
        }
      }
      return true;*/
      double m1 = 8.;
      double m2 = 8.;
      double bound = 20;
      double l1 = 0.2;
      double l2 = 0.2;
      double g = 9.81;
      double T1,T2;
      double q1 = config[0];
      double q2 = config[1];
      double v1 = config[2];
      double v2 = v1 + config[3];
      double a1 = config[4];
      double a2 = a1 + v1*v2 + config[5];


      double x2 = -l1*sin(q1) - l2*sin(q1+q2);
      double z2 = -l1*cos(q1) - l2*cos(q1+q2);
      double l = sqrt(x2*x2+z2*z2);
      double theta = acos(-z2/l);
      std::cout<<"l = "<<l<<" ; theta = "<<theta<<std::endl;

      T1 = m1*l1*sin(q1)*a1 + v1*v1*m1*l1*sin(q1) - l1*m1*g*sin(q1);
      T2 = m2*l2*sin(q2)*a2 + v2*v2*m2*l2*sin(q2) - l2*m2*g*sin(q1+q2);

      double value = T1 + (m2*l*sin(theta)*a2 + v2*v2*m2*l*sin(theta) - l*m2*g*sin(theta));
      std::cout<<"Torque validation, T1 = "<<T1<<"  ; T2 = "<<T2<<" ; T = "<<value<<std::endl;
      if (std::fabs(value) > bound) {
        TorqueBoundValidationReportPtr_t report
            (new TorqueBoundValidationReport (bound,std::fabs(value)));
        validationReport = report;
        return false;
      }
      return true;
    }
    
    TorqueBoundValidation::TorqueBoundValidation (const DevicePtr_t& robot) :
      robot_ (robot)
    {
    }
  } // namespace core
} // namespace hpp
