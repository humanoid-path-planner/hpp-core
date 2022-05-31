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

#ifndef HPP_CORE_WEIGHED_DISTANCE_HH
#define HPP_CORE_WEIGHED_DISTANCE_HH

#include <hpp/core/distance.hh>

namespace hpp {
namespace core {
/// \addtogroup steering_method
/// \{

/// Weighed distance between configurations
///
/// Euclidean distance between configurations seen as vectors.
/// Each degree of freedom is weighed by a positive value.
class HPP_CORE_DLLAPI WeighedDistance : public Distance {
 public:
  static WeighedDistancePtr_t createFromProblem(
      const ProblemConstPtr_t& problem);
  static WeighedDistancePtr_t create(const DevicePtr_t& robot);
  static WeighedDistancePtr_t createWithWeight(const DevicePtr_t& robot,
                                               const vector_t& weights);
  static WeighedDistancePtr_t createCopy(const WeighedDistancePtr_t& distance);
  virtual DistancePtr_t clone() const;
  /// Get weight of joint at given rank
  /// \param rank rank of the joint in robot joint vector
  value_type getWeight(size_type rank) const;
  /// Set weight of joint at given rank
  /// \param rank rank of the joint in robot joint vector
  void setWeight(size_type rank, value_type weight);
  /// Get weights
  const vector_t& weights() const;
  /// Set weights
  void weights(const vector_t& ws);
  /// Get size of weight vector
  size_type size() const { return weights_.size(); }

  /// Get robot
  const DevicePtr_t& robot() const { return robot_; }

 protected:
  WeighedDistance(const ProblemConstPtr_t& problem);
  WeighedDistance(const DevicePtr_t& robot);
  WeighedDistance(const DevicePtr_t& robot, const vector_t& weights);
  WeighedDistance(const WeighedDistance& distance);
  void init(WeighedDistanceWkPtr_t self);
  /// Derived class should implement this function
  virtual value_type impl_distance(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const;
  /// For serialization only.
  WeighedDistance() {}

 private:
  void computeWeights();
  DevicePtr_t robot_;
  vector_t weights_;
  WeighedDistanceWkPtr_t weak_;

  HPP_SERIALIZABLE();
};  // class WeighedDistance
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_WEIGHED_DISTANCE_HH
