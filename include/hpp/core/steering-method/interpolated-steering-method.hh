// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_INTERPOLATED_HH
#define HPP_CORE_STEERING_METHOD_INTERPOLATED_HH

#include <hpp/core/interpolated-path.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method/fwd.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
namespace core {
namespace steeringMethod {
/// \addtogroup steering_method
/// \{

/// Steering method that creates StraightPath instances
///
class HPP_CORE_DLLAPI Interpolated : public SteeringMethod {
 public:
  /// Create instance and return shared pointer
  static InterpolatedPtr_t create(const DevicePtr_t& device) {
    Interpolated* ptr = new Interpolated(device);
    InterpolatedPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  static InterpolatedPtr_t create(const DevicePtr_t& device,
                                  const WeighedDistancePtr_t& distance) {
    Interpolated* ptr = new Interpolated(device, distance);
    InterpolatedPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  static InterpolatedPtr_t createCopy(const InterpolatedPtr_t& other) {
    Interpolated* ptr = new Interpolated(*other);
    InterpolatedPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  virtual SteeringMethodPtr_t copy() const { return createCopy(weak_.lock()); }

  /// create a path between two configurations
  virtual PathPtr_t impl_compute(ConfigurationIn_t q1,
                                 ConfigurationIn_t q2) const {
    value_type length = (*distance_)(q1, q2);
    PathPtr_t path =
        InterpolatedPath::create(device_.lock(), q1, q2, length, constraints());
    return path;
  }

 protected:
  /// Constructor with robot
  /// Weighed distance is created from robot
  Interpolated(const DevicePtr_t& device)
      : SteeringMethod(),
        device_(device),
        distance_(WeighedDistance::create(device)),
        weak_() {}

  /// Constructor with weighed distance
  Interpolated(const DevicePtr_t& device, const WeighedDistancePtr_t& distance)
      : SteeringMethod(), device_(device), distance_(distance), weak_() {}

  /// Copy constructor
  Interpolated(const Interpolated& other)
      : SteeringMethod(other),
        device_(other.device_),
        distance_(other.distance_),
        weak_() {}

  /// Store weak pointer to itself
  void init(InterpolatedWkPtr_t weak) {
    SteeringMethod::init(weak);
    weak_ = weak;
  }

 private:
  DeviceWkPtr_t device_;
  WeighedDistancePtr_t distance_;
  InterpolatedWkPtr_t weak_;
};  // Interpolated
/// \}
}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_STEERING_METHOD_STRAIGHT_HH
