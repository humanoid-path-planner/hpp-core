// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/core/steering-method/steering-kinodynamic.hh>

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>

# include <hpp/core/kinodynamic-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      PathPtr_t Kinodynamic::impl_compute (ConfigurationIn_t q1,
                                           ConfigurationIn_t q2) const
      {
        value_type length = (*problem_.distance()) (q1, q2);
        KinodynamicPathPtr_t path = KinodynamicPath::create (device_.lock (), q1, q2,length/* ... */);
        // TODO
        return path;
      }
      
      
      
      
      
      Kinodynamic::Kinodynamic (const Problem& problem) :
        SteeringMethod (problem), device_ (problem.robot ()), weak_ ()
      {
        if(problem->robot()->extraConfigSpace().dimension() < problem->robot()->numberDof()){
          std::cout<<"Error : you need at least "<<problem->robot()->numberDof()<<" extra DOF"<<std::endl;
          hppDout(error,"you need at least "<<problem->robot()->numberDof()<<" extra DOF");
        }
        aMax_ = 0.5;
        vMax_ = 1;
      }
      
      /// Copy constructor
      Kinodynamic::Kinodynamic (const Kinodynamic& other) :
        SteeringMethod (other), device_ (other.device_)
      {
      }
      
      double Kinodynamic::computeMinTime(double p1,double p2, double v1, double v2, int& sigma, double& t1, double& tv, double& t2){
        // compute the sign of each acceleration
        double deltaPacc = 0.5*(v1-v2)*(fabs(v2-v1)/aMax_);
        sigma = sgn(p2-p1-deltaPacc);
        double a1 = sigma*aMax_;
        double a2 = -a1;
        double vLim = sigma * vMax_;
        
        // test if two segment trajectory is valid :
        bool twoSegment = false;        
        double minT1 = std::max(0.,((v2-v1)/a2));  //lower bound for valid t1 value
        // solve quadratic equation
        double delta = 4*v1*v1 - 4*a1*(((v2*v2-v1*v1)/(2*a2)) - (p2-p1));
        if(delta < 0 )
          std::cout<<"Error : determinant of quadratic function negative"<<std::endl;
        double x1 = (-2*v1 + sqrt(delta))/(2*a1);
        double x2 = (-2*v1 - sqrt(delta))/(2*a1);
        if(x1 > minT1){
          twoSegment = true;
          t1 = x1;
        }
        if(x2 > minT1){
          twoSegment = true;
          t1 = x2;
        }
        if(twoSegment){ // check if max velocity is respected
          if((v1+t1*a1) > vLim)
            twoSegment = false;
        }
        
        if(twoSegment){ // compute t2 for two segment trajectory
          tv = 0.;
          t2 = ((v2-v1)/a2) + t1;
        }else{// compute 3 segment trajectory, with constant velocity phase :
          t1 = (vLim - v1)/a1;
          tv = (v1*v1+v2*v2-2*vLim*vLim)/(2*vLim*a1) + (p2-p1)/vLim ;
          t2 = (v2-vLim)/a2;
        }
        
        double T = t1+tv+t2;
        return T;
      }
      
      
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
