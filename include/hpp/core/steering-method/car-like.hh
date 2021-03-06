//
// Copyright (c) 2017 CNRS
// Authors: Florent Lamiraux
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
