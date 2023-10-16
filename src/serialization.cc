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

#include <boost/serialization/version.hpp>
#if BOOST_VERSION / 100 % 1000 == 74
#include <boost/serialization/library_version_type.hpp>
#endif
// ref https://github.com/boostorg/serialization/issues/219

#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/core/connected-component.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/serialization.hh>
#include <pinocchio/serialization/eigen.hpp>

namespace hpp {
namespace core {

template <typename Archive>
inline void Node::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  Configuration_t& config(configuration_);
  serialization::remove_duplicate::serialize_vector(ar, "config", config,
                                                    version);
  ar& BOOST_SERIALIZATION_NVP(outEdges_);
  ar& BOOST_SERIALIZATION_NVP(inEdges_);
  ar& BOOST_SERIALIZATION_NVP(connectedComponent_);
}
HPP_SERIALIZATION_IMPLEMENT(Node);

template <typename Archive>
inline void Edge::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  ar& BOOST_SERIALIZATION_NVP(n1_);
  ar& BOOST_SERIALIZATION_NVP(n2_);
  ar& BOOST_SERIALIZATION_NVP(path_);
}
HPP_SERIALIZATION_IMPLEMENT(Edge);

template <typename Archive>
inline void ConnectedComponent::serialize(Archive& ar,
                                          const unsigned int version) {
  (void)version;
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(reachableFrom_);
  ar& BOOST_SERIALIZATION_NVP(reachableTo_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(ConnectedComponent);

template <typename Archive>
inline void Roadmap::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  ar& boost::serialization::make_nvp("distance_",
                                     const_cast<DistancePtr_t&>(distance_));
  ar& BOOST_SERIALIZATION_NVP(connectedComponents_);
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(edges_);
  ar& BOOST_SERIALIZATION_NVP(initNode_);
  ar& BOOST_SERIALIZATION_NVP(goalNodes_);
  ar& BOOST_SERIALIZATION_NVP(nearestNeighbor_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(Roadmap);

}  //   namespace core
}  // namespace hpp
