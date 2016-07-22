// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
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
        double Tmax = 0;
        double T = 0;
        double t1,tv,t2,a1;
        int sigma;
        
        size_type configSize = problem_->robot()->configSize() - problem_->robot()->extraConfigSpace().dimension ();
        // looking for Tmax
        hppDout(notice,"## Looking for Tmax :");
        /*const JointVector_t& jv (problem_->robot()->getJointVector ());
        for (model::JointVector_t::const_iterator itJoint = jv.begin (); itJoint != jv.end (); itJoint++) {
          size_type indexConfig = (*itJoint)->rankInConfiguration ();
          size_type indexVel = (*itJoint)->rankInVelocity() + configSize;
          hppDout(notice,"For joint "<<(*itJoint)->name());
          if((*itJoint)->configSize() >= 1){
            T = computeMinTime(q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&sigma,&t1,&tv,&t2);
            if(T > Tmax)
              Tmax = T;
          }
          
        }// for all joints*/
        
        for(int indexConfig = 0 ; indexConfig < configSize ; indexConfig++){
          size_type indexVel = indexConfig + configSize;
          hppDout(notice,"For joint :"<<problem_->robot()->getJointAtConfigRank(indexConfig)->name());
          if(problem_->robot()->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"){
            T = computeMinTime(q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&sigma,&t1,&tv,&t2);
            if(T > Tmax)
              Tmax = T;
          }else{
            hppDout(notice,"!! Steering method for quaternion not implemented yet.");
          }
          
        }
        value_type length = Tmax;     
        // create array of times intervals and acceleration values: 
        Configuration_t a1_t(configSize);
        Configuration_t t1_t(configSize);
        Configuration_t t2_t(configSize);
        Configuration_t tv_t(configSize);
        
        // compute trajectory with fixed time T found 
        /*for (model::JointVector_t::const_iterator itJoint = jv.begin (); itJoint != jv.end (); itJoint++) {
          size_type indexConfig = (*itJoint)->rankInConfiguration ();
          size_type indexVel = (*itJoint)->rankInVelocity() + configSize;
          hppDout(notice,"For joint "<<(*itJoint)->name());          
          if((*itJoint)->configSize() >= 1){
            fixedTimeTrajectory(Tmax,q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&a1,&t1,&tv,&t2);
            a1_t[indexConfig]=a1;
            t1_t[indexConfig]=t1;
            tv_t[indexConfig]=tv;
            t2_t[indexConfig]=t2;     
          }
          
        }// for all joints
        */
        for(int indexConfig = 0 ; indexConfig < configSize ; indexConfig++){
          size_type indexVel = indexConfig + configSize;
          hppDout(notice,"For joint :"<<problem_->robot()->getJointAtConfigRank(indexConfig)->name());
          if(problem_->robot()->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"){          
            fixedTimeTrajectory(Tmax,q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&a1,&t1,&tv,&t2);
            a1_t[indexConfig]=a1;
            t1_t[indexConfig]=t1;
            tv_t[indexConfig]=tv;
            t2_t[indexConfig]=t2;  
          }else{
            hppDout(notice,"!! Steering method for quaternion not implemented yet.");
            a1_t[indexConfig]=0;
            t1_t[indexConfig]=0;
            tv_t[indexConfig]=0;
            t2_t[indexConfig]=0;  
          }
        }
        
        KinodynamicPathPtr_t path = KinodynamicPath::create (device_.lock (), q1, q2,length,a1_t,t1_t,tv_t,t2_t,vMax_);        
        return path;
      }
      
      
      
      
      
      Kinodynamic::Kinodynamic (const ProblemPtr_t& problem) :
        SteeringMethod (problem), device_ (problem->robot ()), weak_ ()
      {
        if((2*(problem->robot()->extraConfigSpace().dimension())) < 2*problem->robot()->configSize()){
          std::cout<<"Error : you need at least "<<2*(problem->robot()->configSize()-problem->robot()->extraConfigSpace().dimension())<<" extra DOF"<<std::endl;
          hppDout(error,"Error : you need at least "<<2*(problem->robot()->configSize() - problem->robot()->extraConfigSpace().dimension())<<" extra DOF");
        }
        aMax_ = 0.5;
        vMax_ = 2;
      }
      
      /// Copy constructor
      Kinodynamic::Kinodynamic (const Kinodynamic& other) :
        SteeringMethod (other), device_ (other.device_)
      {
      }
      
      double Kinodynamic::computeMinTime(double p1, double p2, double v1, double v2, int *sigma, double *t1, double *tv, double *t2) const{
        hppDout(info,"p1 = "<<p1<<"  p2 = "<<p2<<"   ; v1 = "<<v1<<"    v2 = "<<v2);        
        // compute the sign of each acceleration
        double deltaPacc = 0.5*(v1+v2)*(fabs(v2-v1)/aMax_);
        *sigma = sgn(p2-p1-deltaPacc);  //TODO bug sigma == 0
        hppDout(info,"sigma = "<<*sigma);
        double a1 = (*sigma)*aMax_;
        double a2 = -a1;
        double vLim = (*sigma) * vMax_;
        hppDout(info,"Vlim = "<<vLim<<"   ;  aMax = "<<aMax_);
        if((p2-p1) == 0. && (v2-v1)==0. ){  
          hppDout(notice,"No movement in this joints, abort.");
          return 0.;
        }
        // test if two segment trajectory is valid :
        bool twoSegment = false;        
        hppDout(info,"inf bound on t1 (from t2 > 0) "<<-((v2-v1)/a2));
        double minT1 = std::max(0.,-((v2-v1)/a2));  //lower bound for valid t1 value
        // solve quadratic equation
        const double a = a1;
        const double b = 2. * v1;
        const double c = (0.5*(v1+v2)*(v2-v1)/a2) - (p2-p1);
        const double delta = b*b - 4*a*c;
        if(delta < 0 )
          std::cout<<"Error : determinant of quadratic function negative"<<std::endl;
        double x1 = (-b + sqrt(delta))/(2*a);
        double x2 = (-b - sqrt(delta))/(2*a);
        double x = std::max(x1,x2);
        hppDout(info,"t1 before vel limit = "<<x);
        if(x > minT1){
          twoSegment = true;
          *t1 = x;
        }
        
        if(twoSegment){ // check if max velocity is respected
          if(std::abs(v1+(*t1)*a1) > vMax_)
            twoSegment = false;
        }
        if(twoSegment){ // compute t2 for two segment trajectory
          *tv = 0.;
          *t2 = ((v2-v1)/a2) + (*t1);
        }else{// compute 3 segment trajectory, with constant velocity phase :
          *t1 = (vLim - v1)/a1;
          *tv = ((v1*v1+v2*v2 - 2*vLim*vLim)/(2*vLim*a1)) + (p2-p1)/vLim ;
          *t2 = (v2-vLim)/a2;
          hppDout(info,"test 1 "<<(v1*v1+v2*v2 - 2*vLim*vLim));
          hppDout(info,"test 2 "<<((v1*v1+v2*v2 - 2*vLim*vLim)/(2*vLim*a1)));
          hppDout(info,"test 3 "<<((p2-p1)/vLim));
          
          
        }
        if(twoSegment){
          hppDout(notice,"Trajectory with 2 segments");
        }else{
          hppDout(notice,"Trajectory with 3 segments");
        }
        hppDout(notice,"a1 = "<<a1<<"  ;  a2 ="<<a2);
        hppDout(notice,"t = "<<(*t1)<<"   ;   "<<(*tv)<<"   ;   "<<(*t2));
        double T = (*t1)+(*tv)+(*t2);
        hppDout(notice,"T = "<<T);
        return T;
      }
      
      void Kinodynamic::fixedTimeTrajectory(double T, double p1, double p2, double v1, double v2, double *a1, double *t1, double *tv, double *t2) const{
        hppDout(info,"p1 = "<<p1<<"  p2 = "<<p2<<"   ; v1 = "<<v1<<"    v2 = "<<v2);
        double v12 = v1+v2;
        double v2_1 = v2-v1;
        double p2_1 = p2-p1;
        hppDout(info,"v12 = "<<v12<<"   ; v21 = "<<v2_1<<"   ; p21 = "<<p2_1);
        
        if(v2_1 == 0 && p2_1 == 0){
          *a1 = 0;
          *t1 = 0;
          *tv = 0;
          *t2 = 0;
          return;
        }
        double a2;
        double delta;
        delta = 4*T*T*(v12*v12+v2_1*v2_1) - 16*T*v12*p2_1 + 16*p2_1*p2_1;
        if(delta < 0 )
          std::cout<<"Error : determinant of quadratic function negative"<<std::endl;
        double b = 2*T*v12-4*p2_1;
        double a_2 = 2*T*T;
        
        double x1= (-b-sqrt(delta))/a_2;
        double x2= (-b+sqrt(delta))/a_2;
        hppDout(info,"b = "<<b<<"   ;  2*a = "<<a_2);
        hppDout(info,"delta = "<<delta<<"    ; x1 = "<<x1 << "   ;  x2 = "<<x2);
        if(std::abs(x1)>std::abs(x2)){
          *a1 = x1;
        }else{
          *a1 = x2;
        }
        a2 = -(*a1);
        
        *t1 = 0.5*((v2_1/(*a1))+T);
        double vLim = sgn((*a1))*vMax_;
        if(std::abs(v1+(*t1)*(*a1)) <= vMax_){  // two segment trajectory
          hppDout(notice,"Trajectory with 2 segments");
          *t2 = T - (*t1);
          *tv = 0.;
        }else{ // three segment trajectory
          hppDout(notice,"Trajectory with 3 segments");
          // adjust acceleration :
          *a1 = ((vLim - v1)*(vLim - v1) + (vLim - v2)*(vLim - v2))/(2*(vLim*T- p2_1));
          a2 = -(*a1);
          // compute new time intervals :
          *t1 = (vLim - v1)/(*a1);
          *tv = (v1*v1+v2*v2-2*vLim*vLim)/(2*vLim*(*a1)) + (p2-p1)/vLim ;
          *t2 = (v2-vLim)/(a2);
        }
        
        
        hppDout(notice,"a1 = "<<*a1<<"  ;  a2 ="<<a2);
        hppDout(notice,"t = "<<(*t1)<<"   ;   "<<(*tv)<<"   ;   "<<(*t2));
        
      }
      
      
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp

