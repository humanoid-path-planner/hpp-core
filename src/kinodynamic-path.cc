// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/kinodynamic-path.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    KinodynamicPath::KinodynamicPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim) :
      parent_t (device,init,end,length),
      a1_(a1),t1_(t1),tv_(tv),t2_(t2),vLim_(vLim)
    {
      assert (device);
      assert (length >= 0);
      assert (!constraints ());
      hppDout(notice,"Create kinodynamic path with values : ");
      hppDout(notice,"a1 = "<<model::displayConfig(a1_));
      hppDout(notice,"t1 = "<<model::displayConfig(t1_));
      hppDout(notice,"tv = "<<model::displayConfig(tv_));
      hppDout(notice,"t2 = "<<model::displayConfig(t2_));
      hppDout(notice,"vLim = "<<model::displayConfig(vLim_));
      
      
    }
    
    KinodynamicPath::KinodynamicPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim,
                                      ConstraintSetPtr_t constraints) :
      parent_t (device,init,end,length,constraints),
      a1_(a1),t1_(t1),tv_(tv),t2_(t2),vLim_(vLim)
    {
      assert (device);
      assert (length >= 0);
    }
    
    KinodynamicPath::KinodynamicPath (const KinodynamicPath& path) :
      parent_t (path),a1_(path.a1_),t1_(path.t1_),tv_(path.tv_),t2_(path.t2_),vLim_(path.vLim_)
    {
    }
    
    KinodynamicPath::KinodynamicPath (const KinodynamicPath& path,
                                      const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints),
      a1_(path.a1_),t1_(path.t1_),tv_(path.tv_),t2_(path.t2_),vLim_(path.vLim_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }
    
    bool KinodynamicPath::impl_compute (ConfigurationOut_t result,
                                        value_type t) const
    {
      assert (result.size() == device()->configSize());
      
      if (t == timeRange ().first || timeRange ().second == 0) {
        result = initial_;
        return true;
      }
      if (t == timeRange ().second) {
        result = end_;
        return true;
      }
      
      size_type configSize = device()->configSize() - device()->extraConfigSpace().dimension ();      
      // const JointVector_t& jv (device()->getJointVector ());
      double v2,t2;
      size_type indexVel;
      size_type indexAcc;
      // straight path for all the joints, except the translations of the base :
      value_type u = t/timeRange ().second;
      if (timeRange ().second == 0)
        u = 0;

      model::interpolate (device_, initial_, end_, u, result);

      for(int id = 0 ; id < 3 ; id++){ // FIX ME : only work for freeflyer (translation part)
      //for (model::JointVector_t::const_iterator itJoint = jv.begin (); itJoint != jv.end (); itJoint++) {
        // size_type id = (*itJoint)->rankInConfiguration ();
        // size_type indexVel = (*itJoint)->rankInVelocity() + configSize;
        indexVel = id + configSize;
        indexAcc = id + configSize + 3;
        
       // hppDout(notice," PATH For joint "<<(*itJoint)->name());
       // hppDout(notice,"PATH for joint :"<<device()->getJointAtConfigRank(id)->name());
        if(device()->getJointAtConfigRank(id)->name() != "base_joint_SO3"){          
          
          //if((*itJoint)->configSize() >= 1){
          // 3 case (each segment of the trajectory) : 
          if(t <= t1_[id]){
          //  hppDout(info,"on  1° segment");
            result[id] = 0.5*t*t*a1_[id] + t*initial_[indexVel] + initial_[id];
            result[indexVel] = t*a1_[id] + initial_[indexVel];
            result[indexAcc] = a1_[id];
          }else if (t <= (t1_[id] + tv_[id]) ){
          //  hppDout(info,"on  constant velocity segment");
            result[id] = 0.5*t1_[id]*t1_[id]*a1_[id] + t1_[id]*initial_[indexVel] + initial_[id] + (t-t1_[id])*vLim_[id];
            result[indexVel] = vLim_[id];
            result[indexAcc] = 0.;

          }else{
          //  hppDout(info,"on  3° segment");
            t2 = t - tv_[id] - t1_[id] ;
            if(tv_[id] > 0 )
              v2 = vLim_[id];
            else
              v2 = t1_[id]*a1_[id] + initial_[indexVel];
            result[id] = 0.5*t1_[id]*t1_[id]*a1_[id] + t1_[id]*initial_[indexVel] + initial_[id] + tv_[id]*vLim_[id] - 0.5*t2*t2*a1_[id] + t2*v2;
            result[indexVel] = v2 - t2 * a1_[id];
            result[indexAcc] = -a1_[id];

          }
        }// if not quaternion joint

     // }// if joint config size > 1
      
     }// for all joints
      
      return true;
    }
    
    PathPtr_t KinodynamicPath::extract (const interval_t& subInterval) const
    throw (projection_error)
    {
      // Length is assumed to be proportional to interval range
      value_type l = fabs (subInterval.second - subInterval.first);
      hppDout(notice,"%% EXTRACT PATH :");      
      bool success;
      Configuration_t q1 ((*this) (subInterval.first, success));
      if (!success) throw projection_error
          ("Failed to apply constraints in KinodynamicPath::extract");
      Configuration_t q2 ((*this) (subInterval.second, success));
      // set acceleration to 0 for initial and end config :
      size_type configSize = device()->configSize() - device()->extraConfigSpace().dimension ();
      q1[configSize+3] = 0.0;
      q1[configSize+4] = 0.0;
      q1[configSize+5] = 0.0;
      q2[configSize+3] = 0.0;
      q2[configSize+4] = 0.0;
      q2[configSize+5] = 0.0;
      hppDout(info,"from : ");
      hppDout(info,"q1 = "<<model::displayConfig(initial_));
      hppDout(info,"q2 = "<<model::displayConfig(end_));
      hppDout(info,"to : ");
      hppDout(info,"q1 = "<<model::displayConfig(q1));
      hppDout(info,"q2 = "<<model::displayConfig(q2));
      if (!success) throw projection_error
          ("Failed to apply constraints in KinodynamicPath::extract");

      if(subInterval.first > subInterval.second){  // reversed path
        hppDout(notice,"%% REVERSE PATH, not implemented yet !");
        std::cout<<"ERROR, you shouldn't call reverse() on a kinodynamic path"<<std::endl;
        return PathPtr_t();
      }
      double ti,tf,oldT2,oldT1,oldTv;
      hppDout(notice,"%% subinterval PATH");
      // new timebounds
      Configuration_t t1(t1_);
      Configuration_t t2(t2_);
      Configuration_t tv(tv_);
      Configuration_t a1(a1_);




      for(int i = 0 ; i < a1_.size() ; ++i){ // adjust times bounds
        ti = subInterval.first - timeRange_.first;
        tf = timeRange_.second - subInterval.second;
        t1[i] = t1_[i] - ti;
        if(t1[i] <= 0){
          t1[i] = 0; 
          ti = ti - t1_[i];
          tv[i] = tv_[i] - ti;
          if(tv[i] <= 0 ){
            tv[i] = 0;
            ti = ti - tv_[i];
            t2[i] = t2_[i] - ti;
            if(t2[i] <= 0)
              t2[i] = 0;
          }
        }
        oldT1 = t1[i];
        oldT2 = t2[i];
        oldTv = tv[i];
        t2[i] = oldT2 - tf;
        if(t2[i] <= 0 ){
          t2[i] = 0 ;
          tf = tf - oldT2;
          tv[i] = oldTv - tf;
          if(tv[i] <= 0){
            tv[i] = 0;
            tf = tf - oldTv;
            t1[i] = oldT1 - tf;
            if(t1[i] <= 0 )
              t1[i] = 0;
          }
        }


      } // for all joints
      PathPtr_t result = KinodynamicPath::create (device_, q1, q2, l,a1,t1,tv,t2,vLim_,
                                                  constraints ());
      return result;
    }
    
  } //   namespace core
} // namespace hpp

