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
    /// Steering method that creates StraightPath instances
    ///
    class HPP_CORE_DLLAPI SteeringMethodStraight : public SteeringMethod
    {
    public:
      SteeringMethodStraight (const DevicePtr_t& device) :
	SteeringMethod (), device_ (device),
	distance_ (WeighedDistance::create (device))
      {
      }
      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const
      {
        value_type length = (*distance_) (q1, q2);
        PathPtr_t path = StraightPath::create (device_.lock (), q1, q2, length);
        path->constraints (constraints ());
        return path;
      }
    private:
      DeviceWkPtr_t device_;
      WeighedDistancePtr_t distance_;
    }; // SteeringMethodStraight
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_STRAIGHT_HH
