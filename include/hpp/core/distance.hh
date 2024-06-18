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

#ifndef HPP_CORE_DISTANCE_HH
#define HPP_CORE_DISTANCE_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/node.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/serialization-fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup steering_method
/// \{

/// Abstract class for distance between configurations
class HPP_CORE_DLLAPI Distance {
 public:
  value_type operator()(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
    return impl_distance(q1, q2);
  }

  value_type operator()(NodePtr_t n1, NodePtr_t n2) const {
    return impl_distance(n1, n2);
  }

  value_type compute(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
    return impl_distance(q1, q2);
  }

  value_type compute(NodePtr_t n1, NodePtr_t n2) const {
    return impl_distance(n1, n2);
  }

  virtual DistancePtr_t clone() const = 0;

  virtual ~Distance() {};

 protected:
  Distance() {}
  /// Derived class should implement this function
  virtual value_type impl_distance(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const = 0;
  virtual value_type impl_distance(NodePtr_t n1, NodePtr_t n2) const {
    return impl_distance(n1->configuration(), n2->configuration());
  }

  HPP_SERIALIZABLE();
};  // class Distance
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_DISTANCE_HH
