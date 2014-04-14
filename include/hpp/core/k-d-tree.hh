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

namespace hpp {
  namespace core {
	// Built an k-dimentional tree for the nearest neighbour research
    class KDTree
    {
      public:
	DevicePtr_t robot_;
	int dim_;        

	DistancePtr_t distance_;
	Nodes_t nodes_;
	unsigned int bucketSize_;
	
	// number of the splited dimention
	int splitDim_; 
	vector_t upperBounds_;
	vector_t lowerBounds_;
	vector_t loopedDims_;
	
	KDTree* supChild_;
	KDTree* infChild_;

	//constructor
	KDTree(const KDTree* mother);
	KDTree(const DevicePtr_t& robot, const DistancePtr_t& distance_, int bucketSize);

	//destructor
	~KDTree();

	// add a configuration in the KDTree
	void addNode(const NodePtr_t& node);

	// Clear all the nodes in the KDTree
	void clear();

	// Split the node into two subnodes
	void split();

	// find the leaf of the KDtree for the configuration/node. starts the research at KDTree then go down the tree. 
	KDTree* findLeaf(const ConfigurationPtr_t& configuration);
	KDTree* findLeaf(const NodePtr_t& node);

	// find bounds on each dimention
	void findDeviceBounds();

	// distance to the nearest bound on the splited dimention
	value_type distanceOnSplitedDim(const ConfigurationPtr_t& configuration);

	// search nearest node
	NodePtr_t search(const ConfigurationPtr_t& configuration,const ConnectedComponentPtr_t& connectedComponent, 
				value_type& minDistance);
	void search(value_type boxDistance, value_type& minDistance,const ConfigurationPtr_t& configuration,
			const ConnectedComponentPtr_t& connectedComponent, NodePtr_t& nearest);
    };
  }
}
#endif // HPP_CORE_K_D_TREE_HH
