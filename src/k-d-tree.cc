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


# include <hpp/core/k-d-tree.hh>
# include <hpp/core/distance.hh>
# include <hpp/core/node.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/model/device.hh>
#include <iostream>
#include <fstream>

using namespace std;

namespace hpp {
namespace core {

// Constructor with the mother tree node (Child.splitDim_ = mother.splitDim_ + 1, same bounds)
KDTree::KDTree (const KDTreePtr_t mother) : robot_(mother->robot_), dim_(mother->dim_), nodes_(), bucketSize_(mother->bucketSize_),
		splitDim_(), upperBounds_(mother->upperBounds_), lowerBounds_(mother->lowerBounds_), 
		loopedDims_(mother->loopedDims_), supChild_(), infChild_(), distance_(mother->distance_) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "new KDTree" << endl;
	if ( mother->splitDim_ < dim_ ) {	
		splitDim_ = mother->splitDim_ + 1;
	}
	else {
		splitDim_ = 0;
	}
	//for (int i=0 ; i<dim_ ; i++) {	
	//  upperBounds_[i] = mother->upperBounds_[i];
	//  lowerBounds_[i] = mother->lowerBounds_[i];
	//  loopedDims_[i] = mother->loopedDims_[i];
	//}
	supChild_ = NULL;
	infChild_ = NULL;
	debug << "end new KDTree" << endl;
	debug.close();
}

KDTree::KDTree (const DevicePtr_t& robot, const DistancePtr_t& distance, int bucketSize) : robot_(robot), dim_(), nodes_(),
			bucketSize_(bucketSize), splitDim_(), upperBounds_(), lowerBounds_(), loopedDims_(), supChild_(), 
			infChild_(), distance_(distance) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt");
	debug << "findDeviceBounds" << endl;	
	this->findDeviceBounds();
	debug << "end findDeviceBounds" << endl;
	dim_ = lowerBounds_.size();
	splitDim_ = 0;
	supChild_ = NULL;
	infChild_ = NULL;
	debug << "end init root" << endl;

	debug.close();
}

KDTree::~KDTree() {
	delete infChild_;
	delete supChild_;
}

// find the leaf node in the tree for configuration
KDTreePtr_t KDTree::findLeaf (const ConfigurationPtr_t& configuration) {
ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "findLeaf Configuration" << endl;	
	KDTreePtr_t CurrentTree = this;
	while (CurrentTree->supChild_ != NULL && CurrentTree->infChild_ != NULL)  {
		if ( (*configuration) [CurrentTree->supChild_->splitDim_]
				> CurrentTree->supChild_->lowerBounds_[CurrentTree->supChild_->splitDim_] )  {
			CurrentTree = CurrentTree->supChild_;
		}
		else {
			CurrentTree = CurrentTree->infChild_;
		}
	}
	return CurrentTree;
	debug << "end findLeaf Configuration" << endl;
	debug.close();
}

KDTreePtr_t KDTree::findLeaf (const NodePtr_t& node) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "findLeaf node" << endl;
	KDTreePtr_t CurrentTree = this;
	while (CurrentTree->supChild_ != NULL && CurrentTree->infChild_ != NULL)  {
		if ( (*(node->configuration()))[CurrentTree->supChild_->splitDim_]
				> CurrentTree->supChild_->lowerBounds_[CurrentTree->supChild_->splitDim_] )  {
			CurrentTree = CurrentTree->supChild_;
		}
		else {
			CurrentTree = CurrentTree->infChild_;
		}
	}
	debug << "end findLeaf node" << endl;
	debug.close();
	return CurrentTree;	
}


void KDTree::addNode (const NodePtr_t& node) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "add node" << endl;
	KDTreePtr_t Leaf = this->findLeaf(node);
	if ( Leaf->nodes_.size() < bucketSize_ ) {
		Leaf->nodes_.push_front(node);
	}
	else {
		Leaf->split();
		KDTreePtr_t Child;
		for (Nodes_t::iterator it = Leaf->nodes_.begin (); it != Leaf->nodes_.end (); it++ ) {
			Child = Leaf->findLeaf(*it);
			Child->addNode(*it);
		}
		Leaf->nodes_.clear();

		Child = Leaf->findLeaf(node);
		Child->addNode(node);
	}
	debug << "end addnode" << endl;
	debug.close();
}


void KDTree::clear() {
	nodes_.clear();
	if ( infChild_ != NULL ) { infChild_->clear(); }
	if ( supChild_ != NULL ) { supChild_->clear(); }	  
}


void KDTree::split() {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "split" << endl;
	if ( infChild_ != NULL || supChild_ != NULL ) {
		// Error, you're triing to split a non leaf part of the KDTree
		throw std::runtime_error ("Attempt to split the KDTree in a non leaf part");
	}
	else {  debug << "split : new children" << endl;
		infChild_ = new KDTree(this);
		infChild_->upperBounds_[infChild_->splitDim_] = (upperBounds_[infChild_->splitDim_] +
				lowerBounds_[infChild_->splitDim_])/2;
		supChild_ = (new KDTree(this));
		supChild_->lowerBounds_[supChild_->splitDim_] = (upperBounds_[supChild_->splitDim_] +
				lowerBounds_[supChild_->splitDim_])/2;
		debug << "end split : new children" << endl;
	}
	debug << "end split" << endl;
	debug.close();
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
			loopedDims_.conservativeResize(loopedDims_.innerSize() + 1 );
			if ( (*itJoint)->configSize () == 4 ) {
				//We assume if configDize == 4, then the current joint is a SO3Joint
				upperBounds_[i] = 1.;
				lowerBounds_[i] = -1.;
				loopedDims_[i] = 0.;
			}
			else {
				if ( (*itJoint)->isBounded(rank) ) {
					upperBounds_[i] = (*itJoint)->upperBound(rank);
					lowerBounds_[i] = (*itJoint)->lowerBound(rank);
					loopedDims_[i] = 0.;
				}
				else {
					// if unbounded => rotation
					upperBounds_[i] = M_PI;
					lowerBounds_[i] = -M_PI;
					loopedDims_[i] = 1.;
				}
			}
			i++;
		}
	}
}


value_type KDTree::distanceOnSplitedDim (const ConfigurationPtr_t& configuration) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "distanceOnSplitedDim" << endl;
	value_type DistanceToLowerBound = fabs ( lowerBounds_[splitDim_] - (*configuration)[splitDim_] );
	value_type DistanceToUpperBound = fabs ( upperBounds_[splitDim_] - (*configuration)[splitDim_] );
	value_type minDistance = std::min( DistanceToLowerBound, DistanceToUpperBound );	
	

	if ( loopedDims_[splitDim_] == 1. ) {
		// Distance for looped dimentions
		DistanceToLowerBound = fabs ( lowerBounds_[splitDim_] + M_PI ) + fabs ( (*configuration)[splitDim_] - M_PI );
		DistanceToUpperBound = fabs ( upperBounds_[splitDim_] - M_PI ) + fabs ( (*configuration)[splitDim_] + M_PI );
		minDistance = std::min( minDistance, DistanceToLowerBound);
		minDistance = std::min( minDistance, DistanceToUpperBound);	
	}

	debug << "end distanceOnSplitedDim" << endl;
	debug.close();
	return minDistance;
}

NodePtr_t KDTree::search (const ConfigurationPtr_t& configuration, const ConnectedComponentPtr_t& connectedComponent, 
				value_type& minDistance) {
	ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "root search" << endl;
	// We assume that the root KDTree contains the configuration (the root should contain the whole space)
	value_type boxDistance = 0.;	
	NodePtr_t nearest = NULL;	
	minDistance = std::numeric_limits <value_type>::infinity ();
	this->search (boxDistance, minDistance, configuration, connectedComponent, nearest);
	debug << "end rootsearch" << endl;
	debug << nearest << endl;
	debug.close();
	return nearest;
}

void KDTree::search (value_type boxDistance, value_type& minDistance,const ConfigurationPtr_t& configuration,
		const ConnectedComponentPtr_t& connectedComponent, NodePtr_t& nearest) {
ofstream debug;
	debug.open("/local/mgeisert/debug.txt",ios::app);
	debug << "search" << endl;
	if ( boxDistance < minDistance*minDistance ) { // minDistance^2 because boxDistance is a squared distance
		if ( infChild_ == NULL || supChild_ == NULL ) {
			value_type distance = std::numeric_limits <value_type>::infinity ();
			for (Nodes_t::iterator itNode = nodes_.begin ();
					itNode != nodes_.end (); itNode ++) {
				if ( (*itNode)->connectedComponent() == connectedComponent ) {
					distance = (*distance_) (*configuration,
							*((*itNode)->configuration ()));
					if (distance < minDistance) {
						minDistance = distance;
						nearest = (*itNode);
					}
				}
			}
		}
		else {
			// find config to boxes distances
			value_type distanceToInfChild;
			value_type distanceToSupChild;
			if ( boxDistance == 0. ) {
				if ( (*configuration) [supChild_->splitDim_] > lowerBounds_[supChild_->splitDim_])  {
					distanceToSupChild = 0.;
					distanceToInfChild = infChild_->distanceOnSplitedDim(configuration);
				}
				else {
					distanceToInfChild = 0.;
					distanceToSupChild = supChild_->distanceOnSplitedDim(configuration);
				}
			}
			else {
				distanceToInfChild = infChild_->distanceOnSplitedDim(configuration);
				distanceToSupChild = supChild_->distanceOnSplitedDim(configuration);
			}
			// search in the children
			if ( distanceToInfChild < distanceToSupChild ) {
				infChild_->search(boxDistance, minDistance, configuration, connectedComponent, nearest);
				supChild_->search(boxDistance - distanceToInfChild*distanceToInfChild + 							distanceToSupChild*distanceToSupChild, minDistance, configuration,
				connectedComponent, nearest );
			}
			else {
				supChild_->search(boxDistance,minDistance, configuration, connectedComponent, nearest);
				infChild_->search(boxDistance - distanceToSupChild*distanceToSupChild + 							distanceToInfChild*distanceToInfChild, minDistance, configuration,
				connectedComponent, nearest);
			}
		}
	debug << "end search" << endl;
	debug.close();
	}
}
}
}


























