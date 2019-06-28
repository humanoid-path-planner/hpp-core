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


#ifndef HPP_CORE_KINODYNAMIC_ORIENTED_PATH_HH
#define HPP_CORE_KINODYNAMIC_ORIENTED_PATH_HH

#include <hpp/core/kinodynamic-path.hh>

namespace hpp{
  namespace core{
    /// This class is similar to \link hpp::core::KinodynamicPath KinodynamicPath \endlink exept that the orientation of the robot always follow the direction of the velocity.
    /// If the problem parameter "Kinodynamic/forceYawOrientation" have been set to True, only the orientation around the z axis is set to follow the direction of the velocity.

    class HPP_CORE_DLLAPI KinodynamicOrientedPath : public KinodynamicPath
    {
    public :
      typedef KinodynamicPath parent_t;
      virtual ~KinodynamicOrientedPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      static KinodynamicOrientedPathPtr_t create (const DevicePtr_t& device,
                                          ConfigurationIn_t init,
                                          ConfigurationIn_t end,
                                          value_type length,ConfigurationIn_t a1,ConfigurationIn_t t0,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim, bool ignoreZValue = false)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (device, init, end, length,a1,t0,t1,tv,t2,vLim,ignoreZValue);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      /// \param constraints the path is subject to
      static KinodynamicOrientedPathPtr_t create (const DevicePtr_t& device,
                                          ConfigurationIn_t init,
                                          ConfigurationIn_t end,
                                          value_type length,ConfigurationIn_t a1,ConfigurationIn_t t0,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim,
                                          ConstraintSetPtr_t constraints, bool ignoreZValue = false)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (device, init, end, length,a1,t0,t1,tv,t2,vLim,
                                                    constraints,ignoreZValue);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static KinodynamicOrientedPathPtr_t createCopy (const KinodynamicOrientedPathPtr_t& path)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (*path);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      static KinodynamicOrientedPathPtr_t create (const KinodynamicPathPtr_t& path,bool ignoreZValue = false)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (*path,ignoreZValue);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static KinodynamicOrientedPathPtr_t createCopy
      (const KinodynamicOrientedPathPtr_t& path, const ConstraintSetPtr_t& constraints)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (*path, constraints);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      /// Return a shared pointer to this
      ///
      /// As StaightPath are immutable, and refered to by shared pointers,
      /// they do not need to be copied.
      virtual PathPtr_t copy () const
      {
        return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \pre *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
        return createCopy (weak_.lock (), constraints);
      }

      bool ignoreZValue() const {return ignoreZValue_;}
      void ignoreZValue(bool ignoreZValue){ignoreZValue_ = ignoreZValue;}

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
        os << "KinodynamicOrientedPath:" << std::endl;
        os << "interval: [ " << timeRange ().first << ", "
           << timeRange ().second << " ]" << std::endl;
        os << "initial configuration: " << pinocchio::displayConfig(initial_ )<< std::endl;
        os << "final configuration:   " << pinocchio::displayConfig(end_) << std::endl;
        return os;
      }

      void orienteInitAndGoal(const DevicePtr_t& device);

      /// Constructor
      KinodynamicOrientedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                       ConfigurationIn_t end, value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim,bool ignoreZValue );

      /// Constructor with constraints
      KinodynamicOrientedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                       ConfigurationIn_t end, value_type length,ConfigurationIn_t a1,ConfigurationIn_t t0,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim,
                       ConstraintSetPtr_t constraints,bool ignoreZValue);

      /// Copy constructor
      KinodynamicOrientedPath (const KinodynamicOrientedPath& path);


      /// constructor from KinodynamicPath
      KinodynamicOrientedPath (const KinodynamicPath& path,bool ignoreZValue);

      /// Copy constructor with constraints
      KinodynamicOrientedPath (const KinodynamicOrientedPath& path,
                       const ConstraintSetPtr_t& constraints);

      void init (KinodynamicOrientedPathPtr_t self)
      {
        parent_t::init (self);
        weak_ = self;
        checkPath ();
      }

      virtual bool impl_compute (ConfigurationOut_t result,
                                 value_type t) const;

      virtual PathPtr_t impl_extract (const interval_t& subInterval) const
      throw (projection_error);

    private:
      KinodynamicOrientedPathWkPtr_t weak_;
      bool ignoreZValue_;
    };//class kinodynamic oriented path
  }//namespace core
}//namespace hpp

#endif // HPP_CORE_KINODYNAMIC_ORIENTED_PATH_HH
