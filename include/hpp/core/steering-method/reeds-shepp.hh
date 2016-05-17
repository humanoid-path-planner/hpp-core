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

#ifndef HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH
# define HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH

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

      /// Steering method that creates ReedsSheppPath instances
      ///
      class HPP_CORE_DLLAPI ReedsShepp : public SteeringMethod
      {
        public:
          /// Create an instance
          ///
          /// This constructor assumes that:
          /// - the 2 parameters of the configurations corresponds to the XY
          ///   translation joint,
          /// - the 2 following parameters corresponds to the RZ unbounded
          ///   rotation joint.
          /// Use ReedsShepp::setWheelJoints to set the wheel joints.
          static ReedsSheppPtr_t createWithGuess (const ProblemPtr_t& problem)
          {
            ReedsShepp* ptr = new ReedsShepp (problem);
            ReedsSheppPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Create an instance
          ///
          /// This constructor does no assumption.
          static ReedsSheppPtr_t create (const ProblemPtr_t& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels = std::vector<JointPtr_t>())
          {
            ReedsShepp* ptr = new ReedsShepp (problem, turningRadius,
                xyJoint, rzJoint, wheels);
            ReedsSheppPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static ReedsSheppPtr_t createCopy
            (const ReedsSheppPtr_t& other)
            {
              ReedsShepp* ptr = new ReedsShepp (*other);
              ReedsSheppPtr_t shPtr (ptr);
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

          /// Set the wheels
          /// \param computeRadius when true, the turning radius is computed
          ///        from the position of the wheels.
          void setWheelJoints (const std::vector<JointPtr_t> wheels)
          {
            wheels_ = wheels;
          }

          /// Compute the turning radius.
          ///
          /// The turning radius is the maximum of the turning radius of each
          /// wheel. The turning radius of a wheel is the radius of the circle
          /// defined by:
          /// - its center is on the plane x = 0 in the frame of joint RZ,
          /// - the origin of joint RZ is on the circle,
          /// - the bounds of the joint wheel are saturated.
          void computeRadius ();

        protected:
          /// Constructor
          ReedsShepp (const ProblemPtr_t& problem);

          /// Constructor
          ReedsShepp (const ProblemPtr_t& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels);

          /// Copy constructor
          ReedsShepp (const ReedsShepp& other);

          /// Store weak pointer to itself
          void init (ReedsSheppWkPtr_t weak)
          {
            core::SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          value_type computeAngle(const JointPtr_t wheel) const;

          void guessWheels();

          DeviceWkPtr_t device_;
          // distance between front and rear wheel axes.
          value_type rho_;
          JointPtr_t xy_, rz_;
          std::vector<JointPtr_t> wheels_;
          ReedsSheppWkPtr_t weak_;
      }; // ReedsShepp
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH
