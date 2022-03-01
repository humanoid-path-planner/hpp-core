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

#ifndef HPP_CORE_STEERING_METHOD_CAR_LIKE_HH
# define HPP_CORE_STEERING_METHOD_CAR_LIKE_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method/fwd.hh>
# include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Abstract class that implements various type of trajectories for
      /// carlike vehicles
      ///
      class HPP_CORE_DLLAPI CarLike : public SteeringMethod
      {
      public:

	virtual ~CarLike () {}

	  /// Set the wheels
          void setWheelJoints (const std::vector<JointPtr_t> wheels)
          {
            wheels_ = wheels;
          }

          void turningRadius(const value_type& rho);

          inline value_type turningRadius() const { return rho_; }

        protected:
          /// Constructor
          CarLike (const ProblemConstPtr_t& problem);

          /// Constructor
          CarLike (const ProblemConstPtr_t& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels);

          /// Copy constructor
          CarLike (const CarLike& other);

          /// Store weak pointer to itself
          void init (CarLikeWkPtr_t weak)
          {
            core::SteeringMethod::init (weak);
            weak_ = weak;
          }

          DeviceWkPtr_t device_;
          /// Turning radius
          value_type rho_;
          JointPtr_t xy_, rz_;
          size_type xyId_, rzId_;
          std::vector<JointPtr_t> wheels_;
        private:
          CarLikeWkPtr_t weak_;
      }; // CarLike
      std::vector <JointPtr_t> getWheelsFromParameter
      (const ProblemConstPtr_t& problem, const JointPtr_t& rz);

      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_CAR_LIKE_HH
