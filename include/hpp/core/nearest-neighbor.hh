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

#ifndef HPP_CORE_NEAREST_NEIGHBOR_HH
#define HPP_CORE_NEAREST_NEIGHBOR_HH

#include <hpp/core/fwd.hh>
#include <hpp/util/serialization-fwd.hh>

namespace hpp {
namespace core {
/// Optimization of the nearest neighbor search
class NearestNeighbor {
 public:
  virtual void clear() = 0;
  virtual void addNode(const NodePtr_t& node) = 0;

  /**
   * @brief search Return the closest node of the given configuration
   * @param configuration
   * @param connectedComponent
   * @param distance
   * @param reverse if true, compute distance from given configuration to nodes
   * in roadmap, if false from nodes in roadmap to given configuration
   * @return
   */
  virtual NodePtr_t search(ConfigurationIn_t configuration,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& distance, bool reverse = false) = 0;

  virtual NodePtr_t search(const NodePtr_t& node,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& distance) = 0;

  /// \param[out] distance to the Kth closest neighbor
  /// \return the K nearest neighbors
  virtual Nodes_t KnearestSearch(
      ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent, const std::size_t K,
      value_type& distance) = 0;

  /// \param[out] distance to the Kth closest neighbor
  /// \return the K nearest neighbors
  virtual Nodes_t KnearestSearch(
      const NodePtr_t& node, const ConnectedComponentPtr_t& connectedComponent,
      const std::size_t K, value_type& distance) = 0;

  /// Return the K nearest nodes in the whole roadmap
  /// \param configuration, the configuration to which distance is computed,
  /// \param K the number of nearest neighbors to return
  /// \retval distance to the Kth closest neighbor
  /// \return the K nearest neighbors
  virtual Nodes_t KnearestSearch(ConfigurationIn_t configuration,
                                 const RoadmapPtr_t& roadmap,
                                 const std::size_t K, value_type& distance) = 0;

  /// \return all the nodes closer than \c maxDistance to \c configuration
  /// within \c connectedComponent.
  virtual NodeVector_t withinBall(ConfigurationIn_t configuration,
                                  const ConnectedComponentPtr_t& cc,
                                  value_type maxDistance) = 0;

  // merge two connected components in the whole tree
  virtual void merge(ConnectedComponentPtr_t cc1,
                     ConnectedComponentPtr_t cc2) = 0;

  // Get distance function
  virtual DistancePtr_t distance() const = 0;

  virtual ~NearestNeighbor() {};

 private:
  HPP_SERIALIZABLE();
};  // class NearestNeighbor
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_NEAREST_NEIGHBOR_HH
