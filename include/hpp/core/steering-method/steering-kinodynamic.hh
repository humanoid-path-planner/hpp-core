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

      typedef Eigen::Matrix <value_type, 3, 1>   Vector3;


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
        
        /**
         * @brief computeMinTime compute the minimum time required to go from state (p1,v1) to (p2,v2)
         * @param p1 position at state 1
         * @param p2 position at state 2
         * @param v1 velocity at state 1
         * @param v2 velocity at state 2 
         * @param infInterval : infeasible interval
         * @return T the minimal time
         */
        double computeMinTime(int index,double p1,double p2, double v1, double v2,interval_t *infInterval) const;
        
        /**
        * @brief fixedTimeTrajectory compute the minimum acceleration trajectory for desired time T (1 dimension)
        * @param T lenght of the trajectory
        * @param p1 position at state 1
        * @param p2 position at state 2
        * @param v1 velocity at state 1
        * @param v2 velocity at state 2
        * output :
        * @param a1 acceleration during first phase
        * @param t1 time of the first segment
        * @param tv time of constant velocity segment (can be = 0)
        * @param t2 time of the last segment
        * @return T the minimal time
        */
        virtual void fixedTimeTrajectory(int index,double T, double p1, double p2, double v1, double v2, double *a1, double* t1, double* tv, double* t2,double *vLim) const;
        

        
        inline double sgnenum(double val) const{
          return ((0. < val ) - (val < 0.));
        }

        inline int sgn(double d) const {
          return d >= 0.0 ? 1 : -1;
        }

        inline double sgnf(double d) const {
          return d >= 0.0 ? 1.0 : -1.0;
        }

        void setAmax(Vector3 aMax){aMax_ = aMax;}

        void setVmax(Vector3 vMax){vMax_ = vMax;}

        
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
        Vector3 aMax_;
        Vector3 vMax_;
        DeviceWkPtr_t device_;
        KinodynamicWkPtr_t weak_;
      }; // Kinodynamic
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_KINODYNAMIC_HH
