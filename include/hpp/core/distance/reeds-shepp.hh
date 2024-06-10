//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_DISTANCE_REEDS_SHEPP_HH
#define HPP_CORE_DISTANCE_REEDS_SHEPP_HH

#include <hpp/core/distance.hh>
#include <hpp/core/steering-method/fwd.hh>

namespace hpp {
namespace core {
namespace distance {
/// \addtogroup steering_method
/// \{

/// Reeds and Shepp distance
///
/// Compute the distance between two configurations of a nonholonomic
/// cart-like mobile robot with bounded curvature.
class HPP_CORE_DLLAPI ReedsShepp : public Distance {
 public:
  virtual DistancePtr_t clone() const;
  static ReedsSheppPtr_t create(const ProblemConstPtr_t& problem);
  static ReedsSheppPtr_t create(const ProblemConstPtr_t& problem,
                                const value_type& turningRadius,
                                JointPtr_t xyJoint, JointPtr_t rzJoint,
                                std::vector<JointPtr_t> wheels);

  static ReedsSheppPtr_t createCopy(const ReedsSheppPtr_t& distance);

  void turningRadius(const value_type& rho);

  inline value_type turningRadius() const { return rho_; }

 protected:
  ReedsShepp(const ProblemConstPtr_t& problem);
  ReedsShepp(const ProblemConstPtr_t& problem, const value_type& turningRadius,
             JointPtr_t xyJoint, JointPtr_t rzJoint,
             std::vector<JointPtr_t> wheels);
  ReedsShepp(const ReedsShepp& distance);

  /// Derived class should implement this function
  virtual value_type impl_distance(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const;
  void init(const ReedsSheppWkPtr_t& weak);

 private:
  WeighedDistancePtr_t weighedDistance_;
  DeviceWkPtr_t device_;
  /// Turning radius
  value_type rho_;
  JointPtr_t xy_, rz_;
  size_type xyId_, rzId_;
  std::vector<JointPtr_t> wheels_;
  ReedsSheppWkPtr_t weak_;

  ReedsShepp() {};
  HPP_SERIALIZABLE();
};  // class ReedsShepp
/// \}
}  // namespace distance
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_DISTANCE_REEDS_SHEPP_HH
