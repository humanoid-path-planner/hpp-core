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
    /// Path with 2 segment of constant acceleration or 3 segments with a constant velocity segment
    /// The orientation of the robot always follow the direction of the velocity
    ///
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::model::Joint joint \endlink
    /// they parameterize:
    ///   \li linear interpolation for translation joints, bounded rotation
    ///       joints, and translation part of freeflyer joints,
    ///   \li angular interpolation for unbounded rotation joints,
    ///   \li constant angular velocity for SO(3) part of freeflyer joints.
    ///
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
                                          value_type length,ConfigurationIn_t a1,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (device, init, end, length,a1,t1,tv,t2,vLim);
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
                                          value_type length,ConfigurationIn_t a1,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim,
                                          ConstraintSetPtr_t constraints)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (device, init, end, length,a1,t1,tv,t2,vLim,
                                                    constraints);
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
        ptr->initCopy (shPtr);
        ptr->checkPath ();
        return shPtr;
      }

      static KinodynamicOrientedPathPtr_t createCopy (const KinodynamicPathPtr_t& path)
      {
        KinodynamicOrientedPath* ptr = new KinodynamicOrientedPath (*path);
        KinodynamicOrientedPathPtr_t shPtr (ptr);
        ptr->initCopy (shPtr);
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
        ptr->initCopy (shPtr);
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
      /// \precond *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
        return createCopy (weak_.lock (), constraints);
      }


    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
        os << "KinodynamicOrientedPath:" << std::endl;
        os << "interval: [ " << timeRange ().first << ", "
           << timeRange ().second << " ]" << std::endl;
        os << "initial configuration: " << model::displayConfig(initial_ )<< std::endl;
        os << "final configuration:   " << model::displayConfig(end_) << std::endl;
        return os;
      }
      /// Constructor
      KinodynamicOrientedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                       ConfigurationIn_t end, value_type length, ConfigurationIn_t a1, ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2, ConfigurationIn_t vLim);

      /// Constructor with constraints
      KinodynamicOrientedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                       ConfigurationIn_t end, value_type length,ConfigurationIn_t a1,ConfigurationIn_t t1,ConfigurationIn_t tv,ConfigurationIn_t t2,ConfigurationIn_t vLim,
                       ConstraintSetPtr_t constraints);

      /// Copy constructor
      KinodynamicOrientedPath (const KinodynamicOrientedPath& path);


      /// Copy constructor
      KinodynamicOrientedPath (const KinodynamicPath& path);

      /// Copy constructor with constraints
      KinodynamicOrientedPath (const KinodynamicOrientedPath& path,
                       const ConstraintSetPtr_t& constraints);

      void init (KinodynamicOrientedPathPtr_t self)
      {
        parent_t::init (self);
        weak_ = self;
        checkPath ();
      }

      void initCopy (KinodynamicOrientedPathPtr_t self)
      {
        parent_t::initCopy (self);
        weak_ = self;
      }

      virtual bool impl_compute (ConfigurationOut_t result,
                                 value_type t) const;

    private:
      KinodynamicOrientedPathWkPtr_t weak_;
    };//class kinodynamic oriented path
  }//namespace core
}//namespace hpp

#endif // HPP_CORE_KINODYNAMIC_ORIENTED_PATH_HH
