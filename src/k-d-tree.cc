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

# include <hpp/util/debug.hh>
# include <hpp/core/k-d-tree.hh>
# include <hpp/core/distance.hh>
# include <hpp/core/node.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/model/device.hh>
# include <hpp/core/weighed-distance.hh>
#include <iostream>
#include <fstream>

using namespace std;

namespace hpp {
  namespace core {

    // Constructor with the mother tree node (same bounds)
    KDTree::KDTree (const KDTreePtr_t mother) : 
      robot_(mother->robot_),
      dim_(mother->dim_),
      distance_(mother->distance_),
      nodesMap_(),
      bucketSize_(mother->bucketSize_),
      bucket_(0),
      splitDim_(),
      upperBounds_(mother->upperBounds_),
      lowerBounds_(mother->lowerBounds_),
      typeDims_(mother->typeDims_),
      supChild_(),
      infChild_()
       {

      // Incremental split dimention
      //if ( mother->splitDim_ < dim_ ) {
      //	splitDim_ = mother->splitDim_ + 1;
      //}
      //else {
      //	splitDim_ = 0;
      //}

      // Split the widhtest dimention
      try {
	WeighedDistance& weighedDistance =
          dynamic_cast<WeighedDistance&>( (*distance_) );
	double dimWidth = 0.;
	for (int i=0 ; i < dim_ ; i++ ) {
	  if ( (upperBounds_[i] - lowerBounds_[i])
               *weighedDistance.getWeight(i) > dimWidth ) {
	    dimWidth =  upperBounds_[i] - lowerBounds_[i];
	    splitDim_ = i;
	  }
	}
      }
      catch (std::bad_cast& bc) {
	double dimWidth = 0.;
	for (int i=0 ; i < dim_ ; i++ ) {
	  if ( (upperBounds_[i] - lowerBounds_[i]) > dimWidth ) {
	    dimWidth =  upperBounds_[i] - lowerBounds_[i];
	    splitDim_ = i;
	  }
	}
      }

      supChild_ = NULL;
      infChild_ = NULL;
    }

    KDTree::KDTree (const DevicePtr_t& robot, const DistancePtr_t& distance,
		    int bucketSize) :
      robot_(robot),
      dim_(),
      distance_(distance),
      nodesMap_(),
      bucketSize_(bucketSize),
      bucket_(0),
      splitDim_(),
      upperBounds_(),
      lowerBounds_(), 
      typeDims_(),
      supChild_(),
      infChild_()
       {
      this->findDeviceBounds();
      dim_ = lowerBounds_.size();
      splitDim_ = 0;
      supChild_ = NULL;
      infChild_ = NULL;
    }

    KDTree::~KDTree() {
      if (infChild_ != NULL ) { delete infChild_; }
      if (supChild_ != NULL ) { delete supChild_; }
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
	      map != Leaf->nodesMap_.end() ; map++ ) {
	  for (Nodes_t::iterator it = map->second.begin ();
	       it != map->second.end (); it++ ) {
	    Leaf->addNode(*it);
	  }
	  map->second.clear();
	}
	Leaf->addNode(node);
      }
    }


    void KDTree::clear() {
      nodesMap_.clear();
      if ( infChild_ != NULL ) { infChild_ = NULL; }
      if ( supChild_ != NULL ) { supChild_ = NULL; }
    }


    void KDTree::split() {
      if ( infChild_ != NULL || supChild_ != NULL ) {
	// Error, you're triing to split a non leaf part of the KDTree
	throw std::runtime_error 
	  ("Attempt to split the KDTree in a non leaf part");
      }
      else {  infChild_ = new KDTree(this);
	infChild_->upperBounds_[infChild_->splitDim_] = 
	  (upperBounds_[infChild_->splitDim_] + 
	   lowerBounds_[infChild_->splitDim_])/2;
	supChild_ = (new KDTree(this));
	supChild_->lowerBounds_[supChild_->splitDim_] = 
	  (upperBounds_[supChild_->splitDim_] + 
	   lowerBounds_[supChild_->splitDim_])/2;
      }
    }

    // get joints limits
    void KDTree::findDeviceBounds() {
      JointVector_t jv = robot_->getJointVector ();
      int i=0;
      for (JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); itJoint++) {
	for ( unsigned int rank=0 ; rank<(*itJoint)->configSize () ; rank++ ) {
	  upperBounds_.conservativeResize(upperBounds_.innerSize() + 1 );
	  lowerBounds_.conservativeResize(lowerBounds_.innerSize() + 1 );
	  typeDims_.conservativeResize(typeDims_.innerSize() + 1 );
	  if ( (*itJoint)->configSize () == 4 ) {
	    //We assume if configDize == 4, then the current joint is a SO3Joint
	    upperBounds_[i] = 1.;
	    lowerBounds_[i] = -1.;
	    typeDims_[i] = 2.;
	  }
	  else {
	    if ( (*itJoint)->isBounded(rank) ) {
	      upperBounds_[i] = (*itJoint)->upperBound(rank);
	      lowerBounds_[i] = (*itJoint)->lowerBound(rank);
	      typeDims_[i] = 0.;
	    }
	    else {
	      // if unbounded => rotation
	      upperBounds_[i] = M_PI;
	      lowerBounds_[i] = -M_PI;
	      typeDims_[i] = 1.;
	    }
	  }
	  i++;
	}
      }
    }


    value_type KDTree::distanceToBox (const ConfigurationPtr_t& configuration) {
      value_type minDistance;
      value_type DistanceToUpperBound;
      value_type DistanceToLowerBound;
      // Projection of the configuration on the box
      Configuration_t confbox = *configuration;

      if ( typeDims_[splitDim_] != 2.) {
	// Use the "distance" function
	confbox[splitDim_] = lowerBounds_[splitDim_];
	DistanceToLowerBound = (*distance_) (*configuration, confbox);
	confbox[splitDim_] = upperBounds_[splitDim_];
	DistanceToUpperBound = (*distance_) (*configuration, confbox);
	minDistance = std::min( DistanceToLowerBound, DistanceToUpperBound );
      }
      else {
	try {
	  WeighedDistance& weighedDistance = dynamic_cast<WeighedDistance&>
	    ( (*distance_) );
	  DistanceToLowerBound = fabs ( lowerBounds_[splitDim_] - 
					(*configuration)[splitDim_] )
	    * weighedDistance.getWeight(splitDim_);
	  DistanceToUpperBound = fabs ( upperBounds_[splitDim_] -
					(*configuration)[splitDim_] )
	    * weighedDistance.getWeight(splitDim_);
	  minDistance = std::min( DistanceToLowerBound, DistanceToUpperBound );

	  //if ( typeDims_[splitDim_] == 1. ) {
	  // Distance for looped dimentions 
	  //(looped dimentions are assumed to be rotations)
	  //	DistanceToLowerBound = fabs ( lowerBounds_[splitDim_] + M_PI ) +
	  // fabs ( (*configuration)[splitDim_] - M_PI )
	  //					* WeighedDistance.weights_[i];
	  //	DistanceToUpperBound = fabs ( upperBounds_[splitDim_] - M_PI ) +
	  //fabs ( (*configuration)[splitDim_] + M_PI )
	  //					* WeighedDistance.weights_[i];
	  //	minDistance = std::min( minDistance, DistanceToLowerBound);
	  //	minDistance = std::min( minDistance, DistanceToUpperBound);
	  //}
	}
	catch (std::bad_cast& bc) {
	  throw std::runtime_error ("KDTree::DistanceToBox : Distance for quaternions is only implemented for weighedDistance");
	  hppDout (error, "KDTree::DistanceToBox : Distance for quaternions is only implemented for weighedDistance");
	}
      }
      return minDistance;
    }

    NodePtr_t KDTree::search (const ConfigurationPtr_t& configuration,
			      const ConnectedComponentPtr_t& connectedComponent,
                              value_type& minDistance) {
      // Test if the configuration is in the root box
      for ( int i=0 ; i<dim_ ; i++ ) {
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
      return nearest;
    }

    void KDTree::search (value_type boxDistance, value_type& minDistance,
			 const ConfigurationPtr_t& configuration, 
			 const ConnectedComponentPtr_t& connectedComponent,
			 NodePtr_t& nearest) {
      if ( boxDistance < minDistance*minDistance 
	   && nodesMap_.count(connectedComponent) > 0 ) {
	// minDistance^2 because boxDistance is a squared distance
	if ( infChild_ == NULL || supChild_ == NULL ) {
	  value_type distance = std::numeric_limits <value_type>::infinity ();
	  for (Nodes_t::iterator itNode = 
		 nodesMap_[connectedComponent].begin ();
	       itNode != nodesMap_[connectedComponent].end (); itNode ++) {
	    distance = (*distance_) (*configuration, 
				     *((*itNode)->configuration ()));
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
  }
}
