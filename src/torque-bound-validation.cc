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
      std::cout<<"Create torque validation"<<std::endl;
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
      double m1 = 1.;
      double m2 = 1.;
      double bound1 = 10;
      double bound2 = 1.2; // was 0.2
      double l1 = 0.4;
      double l2 = 0.4;
      double lc1 = 0.2;
      double lc2 = 0.2;
      double g = 9.81;
      double T1,T2;
      double q1 = config[0];
      double q2 = config[1];
      double v1 = config[2];
      double v2 = config[3];
      double a1 = config[4];
      double a2 = config[5];

      //coefs :
      double d11,d12,d21,d22,c121,c211,c221,c112,g1,g2;
      d11 = m1*lc1*lc1 + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*cos(q2));
      d12 = d21 = m2*(lc2*lc2 + l1*lc2*cos(q2));
      d22 = m2*lc2*lc2;

      c121 = c211 = c221 = -m2*l1*lc2*sin(q2);
      c112 = -c221;

      g1 = (m1*lc1 + m2*l1)*g*sin(q1) + m2*lc2*g*sin(q1+q2);
      g2 = m2*lc2*g*sin(q1+q2);

      T1 = d11*a1 + d12*a2 + c121*v1*v2 + c211*v2*v1 + c221*v2*v2 + g1;
      T2 = d21*a1 + d22*a2 + c112*v1*v1 + g2;


      double value = T1;

    //  std::cout<<"l = "<<l<<" ; theta = "<<theta<<std::endl;

      //std::cout<<"Torque validation, T1 = "<<T1<<"  ; T2 = "<<T2<<std::endl;
      if (std::fabs(T1) > bound1) {
        TorqueBoundValidationReportPtr_t report
            (new TorqueBoundValidationReport (bound1,std::fabs(T1)));
        validationReport = report;
        return false;
      }
      if (std::fabs(T2) > bound2) {
        TorqueBoundValidationReportPtr_t report
            (new TorqueBoundValidationReport (bound2,std::fabs(T2)));
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
