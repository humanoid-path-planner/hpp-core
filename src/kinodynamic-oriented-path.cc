// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
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

#include <hpp/core/kinodynamic-oriented-path.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace core {

    void KinodynamicOrientedPath::orienteInitAndGoal(const DevicePtr_t& device){
      // adjust initial and end configs, with correct orientation  (aligned with the velocity):
      size_type configSize = device->configSize() - device->extraConfigSpace().dimension ();
      Eigen::Vector3d vi(initial_[configSize],initial_[configSize+1],initial_[configSize+2]);
      Eigen::Vector3d ve(end_[configSize],end_[configSize+1],end_[configSize+2]);
      hppDout(notice,"OrienteInitAndGoal: ignoreZValue = "<<ignoreZValue_);
      if(ignoreZValue_){
        vi[2] = 0.;
        ve[2] = 0.;
      }
      if(vi.norm() > 0){ // if velocity in the state
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),vi);
        initial_[6]=quat.w();
        initial_[3]=quat.x();
        initial_[4]=quat.y();
        initial_[5]=quat.z();
      }
      if(ve.norm() > 0){ // if velocity in the state
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),ve);
        end_[6]=quat.w();
        end_[3]=quat.x();
        end_[4]=quat.y();
        end_[5]=quat.z();
      }
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim, bool ignoreZValue) :
      parent_t (device,init,end,length,a1,t0,t1,tv,t2,vLim),ignoreZValue_(ignoreZValue)
    {
      orienteInitAndGoal(device);
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim,
                                      ConstraintSetPtr_t constraints, bool ignoreZValue) :
      parent_t (device,init,end,length,a1,t0,t1,tv,t2,vLim,constraints),ignoreZValue_(ignoreZValue)

    {
      orienteInitAndGoal(device);
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicOrientedPath& path) :
      parent_t (path),ignoreZValue_(path.ignoreZValue())
    {
      hppDout(notice,"Create copy called, ignoreZValue = "<<ignoreZValue_);
      orienteInitAndGoal(path.device());
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicPath& path,bool ignoreZValue) :
      parent_t (path),ignoreZValue_(ignoreZValue)
    {
      orienteInitAndGoal(device());
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicOrientedPath& path,
                                      const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints),ignoreZValue_(path.ignoreZValue())
    {
      orienteInitAndGoal(device());
    }

    bool KinodynamicOrientedPath::impl_compute (ConfigurationOut_t result,
                                        value_type t) const
    {
      parent_t::impl_compute(result,t);
      // Re oriente the robot such that he always face the velocity vector
      // FIX ME : assume first joint is freeflyer
      size_type configSize = device()->configSize() - device()->extraConfigSpace().dimension ();
      Eigen::Vector3d v(result[configSize],result[configSize+1],result[configSize+2]);
      if(ignoreZValue_)
        v[2] = 0;
      if(v.norm() > 0){ // if velocity in the state
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),v);
        result[6]=quat.w();
        result[3]=quat.x();
        result[4]=quat.y();
        result[5]=quat.z();
      }

      return true;
    }

    PathPtr_t KinodynamicOrientedPath::impl_extract (const interval_t& subInterval) const{
      PathPtr_t path = parent_t::impl_extract(subInterval);
      KinodynamicPathPtr_t kinoPath = dynamic_pointer_cast<KinodynamicPath>(path);
      if(kinoPath)
        return KinodynamicOrientedPath::create(kinoPath,ignoreZValue_);
      else{
        hppDout(error,"Error while casting path in KinodynamicOrientedPath::extract");
        return PathPtr_t();
      }
    }

  }//namespace core
}//namespace hpp
