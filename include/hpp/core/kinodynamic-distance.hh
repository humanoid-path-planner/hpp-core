//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_KINODYNAMIC_DISTANCE_HH
#define HPP_CORE_KINODYNAMIC_DISTANCE_HH

#include <hpp/core/distance.hh>

namespace hpp {
namespace core {
/// \addtogroup steering_method
/// \{

/// This class computed the Distance between two states as the minimal time
/// required to connect this two states with a "bang-bang" trajectory, given
/// velocity and acceleration bounds.
///
/// This time is the same as the length() of a KinodynamicPath computed between
/// this two states by the steeringMethod::Kinodynamic.
///
/// This class require that the dimension of the extraConfigSpace is at least 6
/// and store the velocity and acceleration of the root.
///

class HPP_CORE_DLLAPI KinodynamicDistance : public Distance {
 public:
  static KinodynamicDistancePtr_t create(const DevicePtr_t& robot);
  static KinodynamicDistancePtr_t createFromProblem(
      const ProblemConstPtr_t& problem);

  static KinodynamicDistancePtr_t createCopy(
      const KinodynamicDistancePtr_t& distance);
  virtual DistancePtr_t clone() const;

  /// Get robot
  const DevicePtr_t& robot() const { return robot_; }

 protected:
  KinodynamicDistance(const DevicePtr_t& robot);
  KinodynamicDistance(const ProblemConstPtr_t& problem);
  KinodynamicDistance(const KinodynamicDistance& distance);
  void init(KinodynamicDistanceWkPtr_t self);
  /// Derived class should implement this function
  virtual value_type impl_distance(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const;

  double computeMinTime(double p1, double p2, double v1, double v2) const;

 private:
  DevicePtr_t robot_;
  double aMax_;
  double vMax_;
  KinodynamicDistanceWkPtr_t weak_;
};  // class KinodynamicDistance
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_WEIGHED_DISTANCE_HH
