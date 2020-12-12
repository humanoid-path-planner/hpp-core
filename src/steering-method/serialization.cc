//
// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <boost/serialization/weak_ptr.hpp>
#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/device.hh>

#include <hpp/core/distance.hh>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>

BOOST_CLASS_EXPORT(hpp::core::steeringMethod::ReedsShepp)

namespace hpp {
namespace core {

template <typename Archive>
inline void SteeringMethod::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & BOOST_SERIALIZATION_NVP(problem_);
  ar & BOOST_SERIALIZATION_NVP(constraints_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}

namespace steeringMethod {

template <typename Archive>
inline void ReedsShepp::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & boost::serialization::make_nvp("base", boost::serialization::base_object
				      <SteeringMethod>(*this));
  ar & BOOST_SERIALIZATION_NVP(weighedDistance_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ReedsShepp);
} // namespace steeringMethod

} //   namespace core
} // namespace hpp
