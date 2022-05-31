//
// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
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

#include <boost/serialization/weak_ptr.hpp>
#include <hpp/core/distance.hh>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/serialization.hh>  // For serialization of Device
#include <hpp/util/serialization.hh>
#include <pinocchio/serialization/eigen.hpp>

BOOST_CLASS_EXPORT(hpp::core::WeighedDistance)
BOOST_CLASS_EXPORT(hpp::core::distance::ReedsShepp)

namespace hpp {
namespace core {

template <typename Archive>
inline void Distance::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  (void)ar;
}

template <typename Archive>
inline void WeighedDistance::serialize(Archive& ar,
                                       const unsigned int version) {
  (void)version;
  ar& boost::serialization::make_nvp(
      "base", boost::serialization::base_object<Distance>(*this));
  ar& BOOST_SERIALIZATION_NVP(robot_);
  ar& BOOST_SERIALIZATION_NVP(weights_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(WeighedDistance);

namespace distance {

template <typename Archive>
inline void ReedsShepp::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  ar& boost::serialization::make_nvp(
      "base", boost::serialization::base_object<Distance>(*this));
  ar& BOOST_SERIALIZATION_NVP(weighedDistance_);
  ar& BOOST_SERIALIZATION_NVP(device_);
  ar& BOOST_SERIALIZATION_NVP(rho_);
  ar& BOOST_SERIALIZATION_NVP(xy_);
  ar& BOOST_SERIALIZATION_NVP(rz_);
  ar& BOOST_SERIALIZATION_NVP(xyId_);
  ar& BOOST_SERIALIZATION_NVP(rzId_);
  ar& BOOST_SERIALIZATION_NVP(wheels_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ReedsShepp);

}  // namespace distance

}  //   namespace core
}  // namespace hpp
