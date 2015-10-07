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

#ifndef HPP_CORE_STEERING_METHOD_INTERPOLATED_HH
# define HPP_CORE_STEERING_METHOD_INTERPOLATED_HH

# include <hpp/core/steering-method.hh>
# include <hpp/core/interpolated-path.hh>
# include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates StraightPath instances
      ///
      class HPP_CORE_DLLAPI Interpolated : public SteeringMethod
      {
        public:
          /// Create instance and return shared pointer
          static InterpolatedPtr_t create (const DevicePtr_t& device)
          {
            Interpolated* ptr = new Interpolated (device);
            InterpolatedPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Create instance and return shared pointer
          static InterpolatedPtr_t create (const DevicePtr_t& device,
              const WeighedDistancePtr_t& distance)
          {
            Interpolated* ptr = new Interpolated (device,
                distance);
            InterpolatedPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static InterpolatedPtr_t createCopy (const InterpolatedPtr_t& other)
          {
            Interpolated* ptr = new Interpolated (*other);
            InterpolatedPtr_t shPtr (ptr);
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
              ConfigurationIn_t q2) const
          {
            value_type length = (*distance_) (q1, q2);
            PathPtr_t path = InterpolatedPath::create (device_.lock (), q1, q2,
                length, constraints ());
            return path;
          }

        protected:
          /// Constructor with robot
          /// Weighed distance is created from robot
          Interpolated (const DevicePtr_t& device) :
            SteeringMethod (), device_ (device),
            distance_ (WeighedDistance::create (device)), weak_ ()
          {}

          /// Constructor with weighed distance
          Interpolated (const DevicePtr_t& device,
              const WeighedDistancePtr_t& distance) :
            SteeringMethod (), device_ (device),
            distance_ (distance), weak_ ()
          {}

          /// Copy constructor
          Interpolated (const Interpolated& other) :
            SteeringMethod (other), device_ (other.device_),
            distance_ (other.distance_), weak_ ()
          {}

          /// Store weak pointer to itself
          void init (InterpolatedWkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          DeviceWkPtr_t device_;
          WeighedDistancePtr_t distance_;
          InterpolatedWkPtr_t weak_;
      }; // Interpolated
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_STRAIGHT_HH
