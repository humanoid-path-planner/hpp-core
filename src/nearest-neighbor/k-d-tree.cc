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

#include <iostream>
#include <fstream>
#include <hpp/util/debug.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/node.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/weighed-distance.hh>
#include "../src/nearest-neighbor/k-d-tree.hh"

using namespace std;

namespace hpp {
  namespace core {
    namespace nearestNeighbor {
    // Constructor with the mother tree node (same bounds)
    KDTree::KDTree (const KDTreePtr_t mother, size_type splitDim) :
      robot_(mother->robot_),
      dim_(mother->dim_),
      distance_(mother->distance_),
      weights_ (mother->weights_),
      nodesMap_(),
      bucketSize_(mother->bucketSize_),
      bucket_(0),
      splitDim_(splitDim),
      upperBounds_(mother->upperBounds_),
      lowerBounds_(mother->lowerBounds_),
      supChild_(0x0),
      infChild_(0x0)
    {
    }

    KDTree::KDTree (const DevicePtr_t& robot, const DistancePtr_t& distance,
		    int bucketSize) :
      robot_(robot),
      dim_(),
      distance_(HPP_DYNAMIC_PTR_CAST (WeighedDistance, distance)),
      weights_ (robot->configSize ()),
      nodesMap_(),
      bucketSize_(bucketSize),
      bucket_(0),
      splitDim_(),
      upperBounds_(),
      lowerBounds_(),
      supChild_(),
      infChild_()
       {
	 JointVector_t jointVector = robot_->getJointVector ();
	 if (!distance_) {
	   // create a weighed distance with unit weighs.
       distance_ = WeighedDistance::createWithWeight
	     (robot_, std::vector <value_type> (jointVector.size (), 1.0));
	 }
	 size_type i=0;
	 // Fill vector of weights. index of vector is configuration coordinate
	 // and not joint rank in robot.
	 for (JointVector_t::const_iterator itJoint = jointVector.begin ();
	      itJoint != jointVector.end (); ++itJoint) {
	   weights_.segment ((*itJoint)->rankInConfiguration (),
			     (*itJoint)->configSize ()).setConstant
         (distance_->getWeight (i));
	   ++i;
	 }
      this->findDeviceBounds();
      dim_ = lowerBounds_.size();
      splitDim_ = 0;
      supChild_ = NULL;
      infChild_ = NULL;
    }

    KDTree::~KDTree() {
      clear ();
    }

    // find the leaf node in the tree for the configuration of the node
    KDTreePtr_t KDTree::findLeaf (const NodePtr_t& node) {
      KDTreePtr_t CurrentTree = this;
      CurrentTree->nodesMap_[node->connectedComponent()];
      while ( CurrentTree->supChild_ != NULL && CurrentTree->infChild_ != NULL)
	{
	  if ( (*(node->configuration()))[CurrentTree->supChild_->splitDim_]
	       > CurrentTree->supChild_->lowerBounds_[CurrentTree->supChild_
						      ->splitDim_] )  {
	    CurrentTree = CurrentTree->supChild_;
	    CurrentTree->nodesMap_[node->connectedComponent()];
	  }
	  else {
	    CurrentTree = CurrentTree->infChild_;
	    CurrentTree->nodesMap_[node->connectedComponent()];
	  }
	}
      return CurrentTree;
    }


    void KDTree::addNode (const NodePtr_t& node) {

      KDTreePtr_t Leaf = this->findLeaf(node);
      if ( Leaf->bucket_ < bucketSize_ ) {
	Leaf->nodesMap_[node->connectedComponent()].push_front(node);
	Leaf->bucket_++;
      }
      else {
	Leaf->split();
	for ( NodesMap_t::iterator map = Leaf->nodesMap_.begin();
	      map != Leaf->nodesMap_.end(); ++map) {
	  for (Nodes_t::iterator it = map->second.begin ();
	       it != map->second.end (); ++it) {
	    Leaf->addNode(*it);
	  }
	  map->second.clear();
	}
	Leaf->addNode(node);
      }
    }


    void KDTree::clear() {
      nodesMap_.clear();
      if (infChild_ != NULL ) {
	delete infChild_;
	infChild_ = NULL;
      }
      if (supChild_ != NULL ) {
	delete supChild_;
	supChild_ = NULL;
      }
    }


    void KDTree::split() {
      if ( infChild_ != NULL || supChild_ != NULL ) {
	// Error, you're triing to split a non leaf part of the KDTree
	throw std::runtime_error
	  ("Attempt to split the KDTree in a non leaf part");
      }
      // Compute actual bounds of node configurations
      vector_t actualLower (robot_->configSize ());
      actualLower.setConstant (+std::numeric_limits <value_type>::infinity ());
      vector_t actualUpper (robot_->configSize ());
      actualUpper.setConstant (-std::numeric_limits <value_type>::infinity ());
      std::vector <value_type> values (bucket_);
      for (NodesMap_t::iterator itCC = nodesMap_.begin();
	   itCC != nodesMap_.end(); ++itCC) {
	Nodes_t nodes = itCC->second;
	for (Nodes_t::const_iterator itNode = nodes.begin ();
	     itNode != nodes.end (); ++itNode) {
	  const Configuration_t& q (*((*itNode)->configuration ()));
	  for (size_type i=0; i < q.size (); ++i) {
	    if (q [i] < actualLower [i]) {
	      actualLower [i] = q [i];
	    }
	    if (q [i] > actualUpper [i]) {
	      actualUpper [i] = q [i];
	    }
	  }
	}
      }
      // Split the widest dimention
      double dimWidth = 0.;
      size_type splitDim;
      for (size_type i=0 ; i < actualUpper.size (); i++) {
	if ((actualUpper [i] - actualLower [i]) * weights_ [i] > dimWidth ) {
	  dimWidth =  actualUpper [i] - actualLower [i];
	  splitDim = i;
	}
      }

      // Compute median value of coordinates
      infChild_ = new KDTree (this, splitDim);
      infChild_->upperBounds_ [splitDim] = (actualUpper [splitDim] +
					    actualLower [splitDim])/2;
      supChild_ = new KDTree(this, splitDim);
      supChild_->lowerBounds_ [splitDim] = (actualUpper [splitDim] +
					    actualLower [splitDim])/2;
    }

    // get joints limits
    void KDTree::findDeviceBounds ()
    {
      JointVector_t jv = robot_->getJointVector ();
      int i=0;
      upperBounds_.resize (robot_->configSize ());
      lowerBounds_.resize (robot_->configSize ());
      for (JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); ++itJoint) {
	for (size_type rank=0 ; rank<(*itJoint)->configSize (); rank++) {
	  upperBounds_[i] = (*itJoint)->upperBound (rank);
	  lowerBounds_[i] = (*itJoint)->lowerBound (rank);
	  ++i;
	}
      }
      const model::ExtraConfigSpace& ecs = robot_->extraConfigSpace ();
      for (size_type rank = 0; rank < ecs.dimension (); ++rank) {
	upperBounds_[i] = ecs.upper (rank);
	lowerBounds_[i] = ecs.lower (rank);
	++i;
      }
    }


    value_type KDTree::distanceToBox (const ConfigurationPtr_t& configuration) {
      value_type minDistance;
      value_type DistanceToUpperBound;
      value_type DistanceToLowerBound;
      // Projection of the configuration on the box
      Configuration_t confbox = *configuration;

      DistanceToLowerBound = fabs (lowerBounds_[splitDim_] -
				   (*configuration) [splitDim_])
	* weights_ [splitDim_];
      DistanceToUpperBound = fabs (upperBounds_[splitDim_] -
				   (*configuration)[splitDim_])
	* weights_ [splitDim_];
      minDistance = std::min (DistanceToLowerBound, DistanceToUpperBound);
      return minDistance;
    }

    NodePtr_t KDTree::search (const ConfigurationPtr_t& configuration,
            const ConnectedComponentPtr_t& connectedComponent,
                              value_type& minDistance, bool reverse) {
      // Test if the configuration is in the root box
      for ( std::size_t i=0 ; i<dim_ ; i++ ) {
	if ( (*configuration)[i] < lowerBounds_[i] || (*configuration)[i]
	     > upperBounds_[i] ) {
	  std::ostringstream oss ("The Configuration isn't in the root box: \n"
				  "  i = ");
	  oss << i << ", lower = " << lowerBounds_[i] << ", config = "
	      << (*configuration)[i] << ", upper = " << upperBounds_[i]
	      << ".";
	  throw std::runtime_error (oss.str ());
	}
      }
      value_type boxDistance = 0.;
      NodePtr_t nearest = NULL;
      minDistance = std::numeric_limits <value_type>::infinity ();
      this->search (boxDistance, minDistance, configuration,
		    connectedComponent, nearest);
      assert (nearest);
      return nearest;
    }

    NodePtr_t KDTree::search (const NodePtr_t& node,
            const ConnectedComponentPtr_t& connectedComponent,
                              value_type& minDistance) {
      return search (node->configuration (), connectedComponent, minDistance);
    }

    Nodes_t KDTree::KnearestSearch (const NodePtr_t&,
        const ConnectedComponentPtr_t&, const std::size_t,
        value_type&)
    {
      assert (false && "K-nearest neighbor in KD-tree: unimplemented features");
    }

    Nodes_t KDTree::KnearestSearch (const ConfigurationPtr_t&,
        const ConnectedComponentPtr_t&, const std::size_t,
        value_type&)
    {
      assert (false && "K-nearest neighbor in KD-tree: unimplemented features");
    }

    void KDTree::search (value_type boxDistance, value_type& minDistance,
			 const ConfigurationPtr_t& configuration,
			 const ConnectedComponentPtr_t& connectedComponent,
       NodePtr_t& nearest,bool reverse) {
      if ( boxDistance < minDistance*minDistance
	   && nodesMap_.count(connectedComponent) > 0 ) {
	// minDistance^2 because boxDistance is a squared distance
	if ( infChild_ == NULL || supChild_ == NULL ) {
	  value_type distance = std::numeric_limits <value_type>::infinity ();
	  for (Nodes_t::iterator itNode =
		 nodesMap_[connectedComponent].begin ();
	       itNode != nodesMap_[connectedComponent].end (); ++itNode) {
      if(reverse)
        distance = (*distance_) (*configuration,*((*itNode)->configuration ()));
      else
        distance = (*distance_) (*((*itNode)->configuration ()),*configuration);
	    if (distance < minDistance) {
	      minDistance = distance;
	      nearest = (*itNode);
	    }
	  }
	}
	else {
	  // find config to boxes distances
	  value_type distanceToInfChild;
	  value_type distanceToSupChild;
	  if ( boxDistance == 0. ) {
	    if ( (*configuration) [supChild_->splitDim_]
		 > supChild_->lowerBounds_[supChild_->splitDim_])  {
	      distanceToSupChild = 0.;
	      distanceToInfChild = infChild_->distanceToBox(configuration);
	    }
	    else {
	      distanceToInfChild = 0.;
	      distanceToSupChild = supChild_->distanceToBox(configuration);
	    }
	  }
	  else {
	    distanceToInfChild = infChild_->distanceToBox(configuration);
	    distanceToSupChild = supChild_->distanceToBox(configuration);
	  }
	  // search in the children
	  if ( distanceToInfChild < distanceToSupChild ) {
	    infChild_->search(boxDistance, minDistance,
			      configuration, connectedComponent, nearest);
	    supChild_->search(boxDistance -
			      distanceToInfChild*distanceToInfChild +
			      distanceToSupChild*distanceToSupChild,
			      minDistance, configuration, connectedComponent,
			      nearest );
	  }
	  else {
	    supChild_->search(boxDistance,minDistance,
			      configuration, connectedComponent, nearest);
	    infChild_->search(boxDistance -
			      distanceToSupChild*distanceToSupChild +
			      distanceToInfChild*distanceToInfChild,
			      minDistance, configuration, connectedComponent,
			      nearest);
	  }
	}
      }
    }

    void KDTree::merge(ConnectedComponentPtr_t cc1,
		       ConnectedComponentPtr_t cc2) {
      nodesMap_[cc1].merge(nodesMap_[cc2]);
      NodesMap_t::iterator it = nodesMap_.find (cc2);
      nodesMap_.erase(it);
      if ( infChild_ != NULL || supChild_ != NULL ) {
	infChild_->merge(cc1, cc2);
	supChild_->merge(cc1, cc2);
      }
    }
    } // namespace nearestNeighbor
  } // namespace core
} // namespace hpp
