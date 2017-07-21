// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_HERMITE_PATH_HH
# define HPP_CORE_HERMITE_PATH_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path
    /// \{

    class HPP_CORE_DLLAPI HermitePath : public Path
    {
    public:
      typedef Path parent_t;

      /// Destructor
      virtual ~HermitePath () throw () {}

      static HermitePathPtr_t create (const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      ConstraintSetPtr_t constraints)
      {
        HermitePath* ptr;
        if (constraints) ptr = new HermitePath (device, init, end, constraints);
        else             ptr = new HermitePath (device, init, end);
        HermitePathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static HermitePathPtr_t createCopy (const HermitePathPtr_t& path)
      {
        HermitePath* ptr = new HermitePath (*path);
        HermitePathPtr_t shPtr (ptr);
        ptr->initCopy (shPtr);
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static HermitePathPtr_t createCopy
        (const HermitePathPtr_t& path, const ConstraintSetPtr_t& constraints)
        {
          HermitePath* ptr = new HermitePath (*path, constraints);
          HermitePathPtr_t shPtr (ptr);
          ptr->initCopy (shPtr);
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

      /// Return the internal robot.
      DevicePtr_t device () const;

      void v0 (const vectorIn_t& speed)
      {
        vs_.col(1) = speed;
        hermiteLength_ = -1;
      }

      void v1 (const vectorIn_t& speed)
      {
        vs_.col(2) = speed;
        hermiteLength_ = -1;
      }

      vector_t v0 () const
      {
        return vs_.col(1);
      }

      vector_t v1 () const
      {
        return vs_.col(2);
      }

      vector_t linearVelocity () const
      {
        return vs_.col(0);
      }

      /// Get the initial configuration
      Configuration_t initial () const
      {
        return initial_;
      }

      /// Get the final configuration
      Configuration_t end () const
      {
        return end_;
      }

      const value_type& hermiteLength () const
      {
        return hermiteLength_;
      }

      void computeHermiteLength ();

      vector_t velocity (const value_type& t) const;

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
        os << "HermitePath:" << std::endl;
        os << "interval: [ " << timeRange ().first << ", "
          << timeRange ().second << " ]" << std::endl;
        os << "initial configuration: " << initial().transpose () << std::endl;
        os << "final configuration:   " << end().transpose () << std::endl;
        return os;
      }

      /// Constructor
      HermitePath (const DevicePtr_t& robot, ConfigurationIn_t init,
            ConfigurationIn_t end);

      /// Constructor with constraints
      HermitePath (const DevicePtr_t& robot, ConfigurationIn_t init,
            ConfigurationIn_t end, ConstraintSetPtr_t constraints);

      /// Copy constructor
      HermitePath (const HermitePath& path);

      /// Copy constructor with constraints
      HermitePath (const HermitePath& path,
            const ConstraintSetPtr_t& constraints);

      void init (HermitePathPtr_t self);

      void initCopy (HermitePathPtr_t self);

      virtual bool impl_compute (ConfigurationOut_t result,
                 value_type param) const;

    private:
      vector_t delta (const value_type& t) const;
      void computeVelocities ();

      DevicePtr_t device_;
      Configuration_t initial_, end_;
      Eigen::Matrix<value_type, Eigen::Dynamic, 3> vs_;
      value_type hermiteLength_;

      HermitePathWkPtr_t weak_;
    }; // class HermitePath
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_HERMITE_PATH_HH
