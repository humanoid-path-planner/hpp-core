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
      
      /// Steering method that creates KinodynamicPath instances.
      /// It produce a "bang-bang" trajectory connecting exactly the two given state (position and velocity)
      /// which respect velocity and acceleration bounds defined by the user (see Problem parameters : Kinodynamic/velocityBound and Kinodynamic/accelerationBound)
      ///
      /// Implementation based on the equation of the article https://ieeexplore.ieee.org/document/6943083
      ///
      class HPP_CORE_DLLAPI Kinodynamic : public SteeringMethod
      {
      public:
        
        
        /// Create an instance
        static KinodynamicPtr_t create (const ProblemConstPtr_t& problem)
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
        virtual void fixedTimeTrajectory(int index,double T, double p1, double p2, double v1, double v2, double *a1,double *t0, double* t1, double* tv, double* t2,double *vLim) const;
        

        void setAmax(Vector3 aMax){aMax_ = aMax;}

        void setVmax(Vector3 vMax){vMax_ = vMax;}

        
      protected:
        
        /// Constructor
        Kinodynamic (const ProblemConstPtr_t& problem);
        
        /// Copy constructor
        Kinodynamic (const Kinodynamic& other);
        
        /// Store weak pointer to itself
        void init (KinodynamicWkPtr_t weak)
        {
          core::SteeringMethod::init (weak);
          weak_ = weak;
        }

        Vector3 aMax_;
        Vector3 vMax_;
        double aMaxFixed_;
        double aMaxFixed_Z_;
        bool synchronizeVerticalAxis_;
        bool orientedPath_;
        bool orientationIgnoreZValue_;
      private:
        DeviceWkPtr_t device_;
        KinodynamicWkPtr_t weak_;
      }; // Kinodynamic
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_KINODYNAMIC_HH
