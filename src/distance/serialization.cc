//
// Copyright (c) 2020 CNRS
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

#include <boost/serialization/weak_ptr.hpp>
#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/serialization.hh> // For serialization of Device

#include <hpp/core/distance.hh>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>

BOOST_CLASS_EXPORT(hpp::core::WeighedDistance)
BOOST_CLASS_EXPORT(hpp::core::distance::ReedsShepp)

namespace hpp {
namespace core {

template <typename Archive>
inline void Distance::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  (void) ar;
}

template <typename Archive>
inline void WeighedDistance::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & boost::serialization::make_nvp("base", boost::serialization::base_object<Distance>(*this));
  ar & BOOST_SERIALIZATION_NVP(robot_);
  ar & BOOST_SERIALIZATION_NVP(weights_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(WeighedDistance);

namespace distance {

template <typename Archive>
inline void ReedsShepp::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & boost::serialization::make_nvp("base", boost::serialization::base_object<Distance>(*this));
  ar & BOOST_SERIALIZATION_NVP(weighedDistance_);
  ar & BOOST_SERIALIZATION_NVP(device_);
  ar & BOOST_SERIALIZATION_NVP(rho_);
  ar & BOOST_SERIALIZATION_NVP(xy_);
  ar & BOOST_SERIALIZATION_NVP(rz_);
  ar & BOOST_SERIALIZATION_NVP(xyId_);
  ar & BOOST_SERIALIZATION_NVP(rzId_);
  ar & BOOST_SERIALIZATION_NVP(wheels_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ReedsShepp);

} // namespace distance

} //   namespace core
} // namespace hpp
