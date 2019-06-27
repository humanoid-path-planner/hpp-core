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

#include <hpp/core/kinodynamic-oriented-path.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace core {

    void KinodynamicOrientedPath::orienteInitAndGoal(const DevicePtr_t& device){
      // adjust initial and end configs, with correct orientation  (aligned with the velocity):
      size_type configSize = device->configSize() - device->extraConfigSpace().dimension ();
      Eigen::Vector3d vi(initial_[configSize],initial_[configSize+1],initial_[configSize+2]);
      Eigen::Vector3d ve(end_[configSize],end_[configSize+1],end_[configSize+2]);
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
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim) :
      parent_t (device,init,end,length,a1,t0,t1,tv,t2,vLim)
    {
      orienteInitAndGoal(device);
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0,ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim,
                                      ConstraintSetPtr_t constraints) :
      parent_t (device,init,end,length,a1,t0,t1,tv,t2,vLim,constraints)

    {
      orienteInitAndGoal(device);
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicOrientedPath& path) :
      parent_t (path)
    {
      orienteInitAndGoal(path.device());
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicPath& path) :
      parent_t (path)
    {
      orienteInitAndGoal(path.device());
    }

    KinodynamicOrientedPath::KinodynamicOrientedPath (const KinodynamicOrientedPath& path,
                                      const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints)
    {
      orienteInitAndGoal(path.device());
    }

    bool KinodynamicOrientedPath::impl_compute (ConfigurationOut_t result,
                                        value_type t) const
    {
      parent_t::impl_compute(result,t);
      // Re oriente the robot such that he always face the velocity vector
      // FIX ME : assume first joint is freeflyer
      size_type configSize = device()->configSize() - device()->extraConfigSpace().dimension ();
      Eigen::Vector3d v(result[configSize],result[configSize+1],result[configSize+2]);
      if(v.norm() > 0){ // if velocity in the state
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),v);
        result[6]=quat.w();
        result[3]=quat.x();
        result[4]=quat.y();
        result[5]=quat.z();
      }

      return true;
    }

    PathPtr_t KinodynamicOrientedPath::impl_extract (const interval_t& subInterval) const
    throw (projection_error){
      PathPtr_t path = parent_t::impl_extract(subInterval);
      KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<KinodynamicPath>(path);
      if(kinoPath)
        return KinodynamicOrientedPath::create(kinoPath);
      else{
        hppDout(error,"Error while casting path in KinodynamicOrientedPath::extract");
        return PathPtr_t();
      }
    }

  }//namespace core
}//namespace hpp
