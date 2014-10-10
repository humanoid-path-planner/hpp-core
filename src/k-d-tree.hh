//
// Copyright (c) 2014 CNRS
// Authors: Mathieu Geisert
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_K_D_TREE_HH
# define HPP_CORE_K_D_TREE_HH

# include <hpp/core/distance.hh>
# include <hpp/core/node.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/model/device.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    // Built an k-dimentional tree for the nearest neighbour research
    class KDTree
    {
      // typedef KDTree* KDTreePtr_t;
    public:

      //constructor
      KDTree(const KDTreePtr_t mother, size_type splitDim);
      KDTree(const DevicePtr_t& robot, const DistancePtr_t& distance_,
	     int bucketSize);

      //destructor
      ~KDTree();

      // add a configuration in the KDTree
      void addNode(const NodePtr_t& node);

      // Clear all the nodes in the KDTree
      void clear();

      // search nearest node
      NodePtr_t search(const ConfigurationPtr_t& configuration,
		       const ConnectedComponentPtr_t& connectedComponent,
		       value_type& minDistance);

      // merge two connected components in the whole tree
      void merge(ConnectedComponentPtr_t cc1, ConnectedComponentPtr_t cc2);

    private:
      DevicePtr_t robot_;
      int dim_;

      WeighedDistancePtr_t distance_;
      vector_t weights_;
      typedef std::map <ConnectedComponentPtr_t, Nodes_t> NodesMap_t;
      NodesMap_t nodesMap_;
      unsigned int bucketSize_;
      unsigned int bucket_;

      // number of the splited dimention
      int splitDim_;
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
      value_type distanceToBox(const ConfigurationPtr_t& configuration);

      // search nearest node
      void search(value_type boxDistance, value_type& minDistance,
		  const ConfigurationPtr_t& configuration,
		  const ConnectedComponentPtr_t& connectedComponent,
		  NodePtr_t& nearest);


    };
  }
}
#endif // HPP_CORE_K_D_TREE_HH
