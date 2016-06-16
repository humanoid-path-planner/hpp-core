//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_KINODYNAMIC_HH
# define HPP_CORE_STEERING_METHOD_KINODYNAMIC_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{
      
      /// Steering method that creates KinodynamicPath instances
      ///
      class HPP_CORE_DLLAPI Kinodynamic : public SteeringMethod
      {
      public:
        
        
        /// Create an instance
        static KinodynamicPtr_t create (const ProblemPtr_t& problem)
        {
          Kinodynamic* ptr = new Kinodynamic (problem);
          KinodynamicPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }
        
        /// Copy instance and return shared pointer
        static KinodynamicPtr_t createCopy
        (const KinodynamicPtr_t& other)
        {
          Kinodynamic* ptr = new Kinodynamic (*other);
          KinodynamicPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }
        
        /// Copy instance and return shared pointer
        virtual SteeringMethodPtr_t copy () const
        {
          return createCopy (weak_.lock ());
        }
        
        /// create a path between two configurations
        virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
                                        ConfigurationIn_t q2) const;
        
        
        
      protected:
        
        /// Constructor
        Kinodynamic (const ProblemPtr_t& problem);
        
        /// Copy constructor
        Kinodynamic (const Kinodynamic& other);
        
        /// Store weak pointer to itself
        void init (KinodynamicWkPtr_t weak)
        {
          core::SteeringMethod::init (weak);
          weak_ = weak;
        }
        
      private:
        
        DeviceWkPtr_t device_;
        KinodynamicWkPtr_t weak_;
      }; // Kinodynamic
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_KINODYNAMIC_HH
