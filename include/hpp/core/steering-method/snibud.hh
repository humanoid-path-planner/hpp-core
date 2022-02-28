//
// Copyright (c) 2017 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_STEERING_METHOD_SNIBUD_HH
# define HPP_CORE_STEERING_METHOD_SNIBUD_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method/car-like.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates backward Dubins paths
      ///
      class HPP_CORE_DLLAPI Snibud : public CarLike
      {
        public:
          /// Create an instance
          ///
          /// This constructor assumes that:
          /// - the 2 parameters of the configurations corresponds to the XY
          ///   translation joint,
          /// - the 2 following parameters corresponds to the RZ unbounded
          ///   rotation joint.
          /// Use CarLike::setWheelJoints to set the wheel joints.
          static SnibudPtr_t createWithGuess (const ProblemConstPtr_t& problem)
          {
            Snibud* ptr = new Snibud (problem);
            SnibudPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Create an instance
          ///
          /// This constructor does no assumption.
          static SnibudPtr_t create (const ProblemConstPtr_t& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels = std::vector<JointPtr_t>())
          {
            Snibud* ptr = new Snibud (problem, turningRadius,
                xyJoint, rzJoint, wheels);
            SnibudPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static SnibudPtr_t createCopy
            (const SnibudPtr_t& other)
            {
              Snibud* ptr = new Snibud (*other);
              SnibudPtr_t shPtr (ptr);
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
          Snibud (const ProblemConstPtr_t& problem);

          /// Constructor
          Snibud (const ProblemConstPtr_t& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels);

          /// Copy constructor
          Snibud (const Snibud& other);

          /// Store weak pointer to itself
          void init (SnibudWkPtr_t weak)
          {
            CarLike::init (weak);
            weak_ = weak;
          }

        private:
          SnibudWkPtr_t weak_;
      }; // Snibud
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_SNIBUD_HH
