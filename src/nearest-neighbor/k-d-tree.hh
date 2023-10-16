//
// Copyright (c) 2014 CNRS
// Authors: Mathieu Geisert
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

#ifndef HPP_CORE_NEAREST_NEIGHBOR_K_D_TREE_HH
#define HPP_CORE_NEAREST_NEIGHBOR_K_D_TREE_HH

#include <hpp/core/distance.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/node.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
namespace core {
namespace nearestNeighbor {
// Built an k-dimentional tree for the nearest neighbour research
class KDTree : public NearestNeighbor {
 public:
  // constructor
  KDTree(const KDTreePtr_t mother, size_type splitDim);
  KDTree(const DevicePtr_t& robot, const DistancePtr_t& distance_,
         int bucketSize);

  // destructor
  virtual ~KDTree();

  // add a configuration in the KDTree
  virtual void addNode(const NodePtr_t& node);

  // Clear all the nodes in the KDTree
  virtual void clear();

  // search nearest node
  virtual NodePtr_t search(ConfigurationIn_t configuration,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& minDistance, bool reverse = false);

  // search nearest node
  virtual NodePtr_t search(const NodePtr_t& configuration,
                           const ConnectedComponentPtr_t& connectedComponent,
                           value_type& minDistance);

  virtual Nodes_t KnearestSearch(
      const NodePtr_t& configuration,
      const ConnectedComponentPtr_t& connectedComponent, const std::size_t K,
      value_type& distance);

  virtual Nodes_t KnearestSearch(
      ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent, const std::size_t K,
      value_type& distance);

  /// Return the K nearest nodes in the whole roadmap
  /// \param configuration, the configuration to which distance is computed,
  /// \param roadmap in which nodes are searched,
  /// \param K the number of nearest neighbors to return
  /// \retval distance to the Kth closest neighbor
  /// \return the K nearest neighbors
  virtual Nodes_t KnearestSearch(ConfigurationIn_t configuration,
                                 const RoadmapPtr_t& roadmap,
                                 const std::size_t K, value_type& distance);

  // merge two connected components in the whole tree
  void merge(ConnectedComponentPtr_t cc1, ConnectedComponentPtr_t cc2);
  // Get distance function
  virtual DistancePtr_t distance() const { return distance_; }

 private:
  DevicePtr_t robot_;
  std::size_t dim_;

  WeighedDistancePtr_t distance_;
  vector_t weights_;
  typedef std::map<ConnectedComponentPtr_t, Nodes_t> NodesMap_t;
  NodesMap_t nodesMap_;
  std::size_t bucketSize_;
  std::size_t bucket_;

  // number of the splited dimention
  std::size_t splitDim_;
  vector_t upperBounds_;
  vector_t lowerBounds_;

  // type of each dimention
  //	0 => bounded dimention
  //	1 => looped dimention
  //	2 => quaternion
  vector_t typeDims_;

  KDTreePtr_t supChild_;
  KDTreePtr_t infChild_;

  // Split the node into two subnodes
  void split();

  // find the leaf of the KDtree for the configuration/node.
  // starts the research at KDTree then go down the tree.
  // also add connectedComopnent of node along the path from tree
  // root to tree leaf
  KDTreePtr_t findLeaf(const NodePtr_t& node);

  // find bounds on each dimention
  void findDeviceBounds();

  // distance to the nearest bound on the splited dimention
  value_type distanceToBox(ConfigurationIn_t configuration);

  // search nearest node
  void search(value_type boxDistance, value_type& minDistance,
              ConfigurationIn_t configuration,
              const ConnectedComponentPtr_t& connectedComponent,
              NodePtr_t& nearest, bool reverse = false);

 private:
  KDTree();
  HPP_SERIALIZABLE();
};  // class KDTree
}  // namespace nearestNeighbor
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_NEAREST_NEIGHBOR_K_D_TREE_HH
