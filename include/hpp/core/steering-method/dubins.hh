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

#ifndef HPP_CORE_STEERING_METHOD_DUBINS_HH
# define HPP_CORE_STEERING_METHOD_DUBINS_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method/fwd.hh>
# include <hpp/core/steering-method/car-like.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates DubinsPath instances
      ///
      class HPP_CORE_DLLAPI Dubins : public CarLike
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
          static DubinsPtr_t createWithGuess (const Problem& problem)
          {
            Dubins* ptr = new Dubins (problem);
            DubinsPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Create an instance
          ///
          /// This constructor does no assumption.
          static DubinsPtr_t create (const Problem& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels = std::vector<JointPtr_t>())
          {
            Dubins* ptr = new Dubins (problem, turningRadius,
                xyJoint, rzJoint, wheels);
            DubinsPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static DubinsPtr_t createCopy
            (const DubinsPtr_t& other)
            {
              Dubins* ptr = new Dubins (*other);
              DubinsPtr_t shPtr (ptr);
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
          Dubins (const Problem& problem);

          /// Constructor
          Dubins (const Problem& problem,
              const value_type turningRadius,
              JointPtr_t xyJoint, JointPtr_t rzJoint,
              std::vector <JointPtr_t> wheels);

          /// Copy constructor
          Dubins (const Dubins& other);

          /// Store weak pointer to itself
          void init (DubinsWkPtr_t weak)
          {
            CarLike::init (weak);
            weak_ = weak;
          }

        private:
          DubinsWkPtr_t weak_;
      }; // Dubins
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_DUBINS_HH
