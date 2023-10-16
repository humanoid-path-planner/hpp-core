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

#ifndef HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH
#define HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/nearest-neighbor.hh>

namespace hpp {
namespace core {
namespace nearestNeighbor {
/// Optimization of the nearest neighbor search
class Basic : public NearestNeighbor {
 public:
  Basic(const DistancePtr_t& distance) : distance_(distance) {}

  ~Basic() {}

  virtual void clear() {}

  void addNode(const NodePtr_t&) {}

  virtual NodePtr_t search(const NodePtr_t& node,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& distance);

  virtual NodePtr_t search(ConfigurationIn_t configuration,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& distance, bool reverse = false);

  virtual Nodes_t KnearestSearch(
      ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent, const std::size_t K,
      value_type& distance);

  virtual Nodes_t KnearestSearch(
      const NodePtr_t& node, const ConnectedComponentPtr_t& connectedComponent,
      const std::size_t K, value_type& distance);

  /// Return the K nearest nodes in the whole roadmap
  /// \param configuration, the configuration to which distance is computed,
  /// \param roadmap in which nodes are searched,
  /// \param K the number of nearest neighbors to return
  /// \retval distance to the Kth closest neighbor
  /// \return the K nearest neighbors
  virtual Nodes_t KnearestSearch(ConfigurationIn_t configuration,
                                 const RoadmapPtr_t& roadmap,
                                 const std::size_t K, value_type& distance);

  NodeVector_t withinBall(ConfigurationIn_t configuration,
                          const ConnectedComponentPtr_t& cc,
                          value_type maxDistance);

  virtual void merge(ConnectedComponentPtr_t, ConnectedComponentPtr_t) {}

  // Get distance function
  virtual DistancePtr_t distance() const { return distance_; }

 private:
  const DistancePtr_t distance_;

  Basic() {}
  HPP_SERIALIZABLE();
};  // class Basic
}  // namespace nearestNeighbor
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH
