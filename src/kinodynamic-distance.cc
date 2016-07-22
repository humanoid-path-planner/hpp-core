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

#include <hpp/core/kinodynamic-distance.hh>
#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/children-iterator.hh>
#include <Eigen/SVD>

namespace hpp {
namespace core {


KinodynamicDistancePtr_t KinodynamicDistance::create (const DevicePtr_t& robot)
{
    KinodynamicDistance* ptr = new KinodynamicDistance (robot);
    KinodynamicDistancePtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
}


KinodynamicDistancePtr_t KinodynamicDistance::createCopy
(const KinodynamicDistancePtr_t& distance)
{
    KinodynamicDistance* ptr = new KinodynamicDistance (*distance);
    KinodynamicDistancePtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
}

DistancePtr_t KinodynamicDistance::clone () const
{
    return createCopy (weak_.lock ());
}



KinodynamicDistance::KinodynamicDistance (const DevicePtr_t& robot) :
    robot_ (robot)
{
    if((2*(robot_->extraConfigSpace().dimension())) < robot_->configSize()){
        std::cout<<"Error : you need at least "<<robot_->configSize()-robot_->extraConfigSpace().dimension()<<" extra DOF"<<std::endl;
        hppDout(error,"Error : you need at least "<<robot_->configSize() - robot_->extraConfigSpace().dimension()<<" extra DOF");
    }
    aMax_ = 0.5;
    vMax_ = 2;
}


KinodynamicDistance::KinodynamicDistance (const KinodynamicDistance& distance) :
    robot_ (distance.robot_)
{
}

void KinodynamicDistance::init (KinodynamicDistanceWkPtr_t self)
{
    weak_ = self;
}

int sgn(double val){
    return ((0. < val ) - (val < 0.));
}

double sgnf(double val){
    return ((0. < val ) - (val < 0.));
}

double KinodynamicDistance::computeMinTime(double p1, double p2, double v1, double v2, int *sigma, double *t1, double *tv, double *t2) const{
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

value_type KinodynamicDistance::impl_distance (ConfigurationIn_t q1,
                                               ConfigurationIn_t q2) const
{
    double Tmax = 0;
    double T = 0;
    double t1,tv,t2;
    int sigma;

    size_type configSize = robot_->configSize() - robot_->extraConfigSpace().dimension ();
    // looking for Tmax
    hppDout(notice,"distance :  Looking for Tmax :");


    for(int indexConfig = 0 ; indexConfig < configSize ; indexConfig++){
        size_type indexVel = indexConfig + configSize;
        hppDout(notice,"For joint :"<<problem_->robot()->getJointAtConfigRank(indexConfig)->name());
        if(robot_->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"){
            T = computeMinTime(q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&sigma,&t1,&tv,&t2);
            if(T > Tmax)
                Tmax = T;
        }else{
            hppDout(notice,"!! Kinodynamic distance for quaternion not implemented yet.");
        }

    }

    return Tmax;
}
} //   namespace core
} // namespace hpp