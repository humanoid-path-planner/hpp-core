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
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/kinodynamic-path.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    KinodynamicPath::KinodynamicPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, double vMax) :
      parent_t (interval_t (0, length), device->configSize (),
                device->numberDof ()),
      device_ (device), initial_ (init), end_ (end),a1_(a1),t1_(t1),tv_(tv),t2_(t2),vMax_(vMax)
    {
      assert (device);
      assert (length >= 0);
      assert (!constraints ());
      hppDout(notice,"Create kinodynamic path with values : ");
      hppDout(notice,"a1 = "<<model::displayConfig(a1_));
      hppDout(notice,"t1 = "<<model::displayConfig(t1_));
      hppDout(notice,"tv = "<<model::displayConfig(tv_));
      hppDout(notice,"t2 = "<<model::displayConfig(t2_));
      hppDout(notice,"vMax = "<<vMax_);
      
      
    }
    
    KinodynamicPath::KinodynamicPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, double vMax,
                                      ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, length), device->configSize (),
                device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end),a1_(a1),t1_(t1),tv_(tv),t2_(t2),vMax_(vMax)
    {
      assert (device);
      assert (length >= 0);
    }
    
    KinodynamicPath::KinodynamicPath (const KinodynamicPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_)
    {
    }
    
    KinodynamicPath::KinodynamicPath (const KinodynamicPath& path,
                                      const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }
    
    bool KinodynamicPath::impl_compute (ConfigurationOut_t result,
                                        value_type param) const
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
      for(int id = 0 ; id < configSize ; id++){
      //for (model::JointVector_t::const_iterator itJoint = jv.begin (); itJoint != jv.end (); itJoint++) {
        // size_type id = (*itJoint)->rankInConfiguration ();
        // size_type indexVel = (*itJoint)->rankInVelocity() + configSize;
        indexVel = id + configSize;
        indexAcc = id + 2*configSize;
        
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
          }else if (t <= t1_[id] + tv_[id] ){
          //  hppDout(info,"on  constant velocity segment");
            result[id] = 0.5*t1_[id]*t1_[id]*a1_[id] + t1_[id]*initial_[indexVel] + initial_[id] + (t-t1_[id])*sgnf(a1_[id])*vMax_;
            result[indexVel] = sgnf(a1_[id])*vMax_;
            result[indexAcc] = 0.;

          }else{
          //  hppDout(info,"on  3° segment");
            t2 = t - tv_[id] - t1_[id] ;
            if(tv_[id] > 0 )
              v2 = sgnf(a1_[id])*vMax_;
            else
              v2 = t1_[id]*a1_[id] + initial_[indexVel];
            result[id] = 0.5*t1_[id]*t1_[id]*a1_[id] + t1_[id]*initial_[indexVel] + initial_[id] + tv_[id]*sgnf(a1_[id])*vMax_ - 0.5*t2*t2*a1_[id] + t2*v2;
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
      hppDout(info,"q1 = "<<model::displayConfig(q1));
      hppDout(info,"q2 = "<<model::displayConfig(q2));      
      if (!success) throw projection_error
          ("Failed to apply constraints in KinodynamicPath::extract");
      Configuration_t t1,t2,tv,a1; // new timebounds
      double ti,tf;      
      if(subInterval.first < subInterval.second){
        hppDout(notice,"%% subinterval PATH");
        t1 = t1_;
        t2 = (t2_);
        tv = (tv_);
        a1 = (a1_);
        ti = subInterval.first;
        tf = length() - subInterval.second;
      }else{  // reversed path
        hppDout(notice,"%% REVERSE PATH, not implemented yet !");
        return PathPtr_t();
      }
      for(int i = 0 ; i < a1_.size() ; ++i){ // adjust times bounds
        t1[i] = t1_[i] - ti;
        if(t1[i] <= 0){
          t1[i] = 0; 
          ti = ti - t1_[i];
          tv[i] = tv_[i] - ti;
          if(tv[1] <= 0 ){
            tv[i] = 0;
            ti = ti - tv_[i]; 
            t2[i] = t2_[i] - ti;
          }
        }
        t2[i] = t2_[i] - tf;
        if(t2[i] <= 0 ){
          t2[i] = 0 ;
          tf = tf - t2_[i];
          tv[i] = tv_[i] - tf;
          if(tv[i] <= 0){
            tv[i] = 0;
            tf = tf - tv_[i];
            t1[i] = t1_[i] - tf;
          }
        }
        
      } // for all joints
      PathPtr_t result = KinodynamicPath::create (device_, q1, q2, l,a1,t1,tv,t2,vMax_,
                                                  constraints ());
      return result;
    }
    
    DevicePtr_t KinodynamicPath::device () const
    {
      return device_;
    }
  } //   namespace core
} // namespace hpp

