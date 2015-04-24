//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_STRAIGHT_HH
# define HPP_CORE_STEERING_METHOD_STRAIGHT_HH

# include <hpp/core/steering-method.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Steering method that creates StraightPath instances
    ///
    class HPP_CORE_DLLAPI SteeringMethodStraight : public SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringMethodStraightPtr_t create (const DevicePtr_t& device)
      {
	SteeringMethodStraight* ptr = new SteeringMethodStraight (device);
	SteeringMethodStraightPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Create instance and return shared pointer
      static SteeringMethodStraightPtr_t create
	(const DevicePtr_t& device, const WeighedDistancePtr_t& distance)
      {
	SteeringMethodStraight* ptr = new SteeringMethodStraight (device,
								  distance);
	SteeringMethodStraightPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringMethodStraightPtr_t createCopy
	(const SteeringMethodStraightPtr_t& other)
      {
	SteeringMethodStraight* ptr = new SteeringMethodStraight (*other);
	SteeringMethodStraightPtr_t shPtr (ptr);
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
        PathPtr_t path = StraightPath::create (device_.lock (), q1, q2, length,
					       constraints ());
        return path;
      }
    protected:
      /// Constructor with robot
      /// Weighed distance is created from robot
      SteeringMethodStraight (const DevicePtr_t& device) :
	SteeringMethod (), device_ (device),
	distance_ (WeighedDistance::create (device)), weak_ ()
      {
      }
      /// Constructor with weighed distance
      SteeringMethodStraight (const DevicePtr_t& device,
			      const WeighedDistancePtr_t& distance) :
	SteeringMethod (), device_ (device),
	distance_ (distance), weak_ ()
      {
      }
      /// Copy constructor
      SteeringMethodStraight (const SteeringMethodStraight& other) :
	SteeringMethod (other), device_ (other.device_),
	distance_ (other.distance_), weak_ ()
      {
      }

      /// Store weak pointer to itself
      void init (SteeringMethodStraightWkPtr_t weak)
      {
	SteeringMethod::init (weak);
	weak_ = weak;
      }
    private:
      DeviceWkPtr_t device_;
      WeighedDistancePtr_t distance_;
      SteeringMethodStraightWkPtr_t weak_;
    }; // SteeringMethodStraight
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_STRAIGHT_HH
