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

#include "../src/nearest-neighbor/k-d-tree.hh"

#include <fstream>
#include <hpp/core/distance.hh>
#include <hpp/core/node.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/debug.hh>
#include <iostream>
#include <pinocchio/multibody/model.hpp>
#include <stdexcept>

using namespace std;

namespace hpp {
namespace core {
namespace nearestNeighbor {
// Constructor with the mother tree node (same bounds)
KDTree::KDTree(const KDTreePtr_t& mother, size_type splitDim)
    : robot_(mother->robot_),
      dim_(mother->dim_),
      distance_(mother->distance_),
      weights_(mother->weights_),
      nodesMap_(),
      bucketSize_(mother->bucketSize_),
      bucket_(0),
      splitDim_(splitDim),
      upperBounds_(mother->upperBounds_),
      lowerBounds_(mother->lowerBounds_),
      supChild_(0x0),
      infChild_(0x0) {}

KDTree::KDTree(const DevicePtr_t& robot, const DistancePtr_t& distance,
               int bucketSize)
    : robot_(robot),
      dim_(),
      distance_(HPP_DYNAMIC_PTR_CAST(WeighedDistance, distance)),
      weights_(robot->configSize()),
      nodesMap_(),
      bucketSize_(bucketSize),
      bucket_(0),
      splitDim_(),
      upperBounds_(),
      lowerBounds_(),
      supChild_(),
      infChild_() {
  if (!distance_) {
    // create a weighed distance with unit weighs.
    vector_t ones(robot_->model().njoints);
    ones.fill(1.0);
    distance_ = WeighedDistance::createWithWeight(robot_, ones);
  }
  // Fill vector of weights. index of vector is configuration coordinate
  // and not joint rank in robot.
  for (int i = 1; i < robot_->model().njoints; ++i) {
    weights_.segment(robot_->model().idx_qs[i], robot_->model().joints[i].nq())
        .setConstant(distance_->getWeight(i - 1));
  }
  this->findDeviceBounds();
  dim_ = lowerBounds_.size();
  splitDim_ = 0;
  supChild_ = NULL;
  infChild_ = NULL;
}

KDTree::~KDTree() { clear(); }

// find the leaf node in the tree for the configuration of the node
KDTreePtr_t KDTree::findLeaf(const NodePtr_t& node) {
  KDTreePtr_t CurrentTree = shared_from_this();
  CurrentTree->nodesMap_[node->connectedComponent()];
  while (CurrentTree->supChild_ != NULL && CurrentTree->infChild_ != NULL) {
    if ((node->configuration())[CurrentTree->supChild_->splitDim_] >
        CurrentTree->supChild_
            ->lowerBounds_[CurrentTree->supChild_->splitDim_]) {
      CurrentTree = CurrentTree->supChild_;
      CurrentTree->nodesMap_[node->connectedComponent()];
    } else {
      CurrentTree = CurrentTree->infChild_;
      CurrentTree->nodesMap_[node->connectedComponent()];
    }
  }
  return CurrentTree;
}

void KDTree::addNode(const NodePtr_t& node) {
  KDTreePtr_t Leaf = this->findLeaf(node);
  if (Leaf->bucket_ < bucketSize_) {
    Leaf->nodesMap_[node->connectedComponent()].push_front(node);
    Leaf->bucket_++;
  } else {
    Leaf->split();
    for (NodesMap_t::iterator map = Leaf->nodesMap_.begin();
         map != Leaf->nodesMap_.end(); ++map) {
      for (Nodes_t::iterator it = map->second.begin(); it != map->second.end();
           ++it) {
        Leaf->addNode(*it);
      }
      map->second.clear();
    }
    Leaf->addNode(node);
  }
}

void KDTree::clear() { nodesMap_.clear(); }

void KDTree::split() {
  if (infChild_ != NULL || supChild_ != NULL) {
    // Error, you're triing to split a non leaf part of the KDTree
    throw std::runtime_error("Attempt to split the KDTree in a non leaf part");
  }
  // Compute actual bounds of node configurations
  vector_t actualLower(robot_->configSize());
  actualLower.setConstant(+std::numeric_limits<value_type>::infinity());
  vector_t actualUpper(robot_->configSize());
  actualUpper.setConstant(-std::numeric_limits<value_type>::infinity());
  std::vector<value_type> values(bucket_);
  for (NodesMap_t::iterator itCC = nodesMap_.begin(); itCC != nodesMap_.end();
       ++itCC) {
    Nodes_t nodes = itCC->second;
    for (Nodes_t::const_iterator itNode = nodes.begin(); itNode != nodes.end();
         ++itNode) {
      const Configuration_t& q((*itNode)->configuration());
      for (size_type i = 0; i < q.size(); ++i) {
        if (q[i] < actualLower[i]) {
          actualLower[i] = q[i];
        }
        if (q[i] > actualUpper[i]) {
          actualUpper[i] = q[i];
        }
      }
    }
  }
  // Split the widest dimention
  double dimWidth = 0.;
  size_type splitDim;
  for (size_type i = 0; i < actualUpper.size(); i++) {
    if ((actualUpper[i] - actualLower[i]) * weights_[i] > dimWidth) {
      dimWidth = actualUpper[i] - actualLower[i];
      splitDim = i;
    }
  }

  // Compute median value of coordinates
  infChild_ = std::make_shared<KDTree>(shared_from_this(), splitDim);
  infChild_->upperBounds_[splitDim] =
      (actualUpper[splitDim] + actualLower[splitDim]) / 2;
  supChild_ = std::make_shared<KDTree>(shared_from_this(), splitDim);
  supChild_->lowerBounds_[splitDim] =
      (actualUpper[splitDim] + actualLower[splitDim]) / 2;
}

// get joints limits
void KDTree::findDeviceBounds() {
  upperBounds_.resize(robot_->configSize());
  lowerBounds_.resize(robot_->configSize());
  int nq = robot_->model().nq;
  upperBounds_.head(nq) = robot_->model().upperPositionLimit;
  lowerBounds_.head(nq) = robot_->model().lowerPositionLimit;
  const pinocchio::ExtraConfigSpace& ecs = robot_->extraConfigSpace();
  upperBounds_.segment(nq, ecs.dimension()) = ecs.upper();
  lowerBounds_.segment(nq, ecs.dimension()) = ecs.lower();
}

value_type KDTree::distanceToBox(ConfigurationIn_t configuration) {
  value_type minDistance;
  value_type DistanceToUpperBound;
  value_type DistanceToLowerBound;
  // Projection of the configuration on the box
  Configuration_t confbox = configuration;

  DistanceToLowerBound =
      fabs(lowerBounds_[splitDim_] - configuration[splitDim_]) *
      weights_[splitDim_];
  DistanceToUpperBound =
      fabs(upperBounds_[splitDim_] - configuration[splitDim_]) *
      weights_[splitDim_];
  minDistance = std::min(DistanceToLowerBound, DistanceToUpperBound);
  return minDistance;
}

NodePtr_t KDTree::search(ConfigurationIn_t configuration,
                         const ConnectedComponentPtr_t& connectedComponent,
                         value_type& minDistance, bool) {
  // Test if the configuration is in the root box
  for (std::size_t i = 0; i < dim_; i++) {
    if (configuration[i] < lowerBounds_[i] ||
        configuration[i] > upperBounds_[i]) {
      std::ostringstream oss(
          "The Configuration isn't in the root box: \n"
          "  i = ");
      oss << i << ", lower = " << lowerBounds_[i]
          << ", config = " << configuration[i]
          << ", upper = " << upperBounds_[i] << ".";
      throw std::runtime_error(oss.str());
    }
  }
  value_type boxDistance = 0.;
  NodePtr_t nearest = NULL;
  minDistance = std::numeric_limits<value_type>::infinity();
  this->search(boxDistance, minDistance, configuration, connectedComponent,
               nearest);
  assert(nearest);
  return nearest;
}

NodePtr_t KDTree::search(const NodePtr_t& node,
                         const ConnectedComponentPtr_t& connectedComponent,
                         value_type& minDistance) {
  return search(node->configuration(), connectedComponent, minDistance);
}

Nodes_t KDTree::KnearestSearch(const NodePtr_t&, const ConnectedComponentPtr_t&,
                               const std::size_t, value_type&) {
  assert(false && "K-nearest neighbor in KD-tree: unimplemented features");
  return Nodes_t();
}

Nodes_t KDTree::KnearestSearch(ConfigurationIn_t,
                               const ConnectedComponentPtr_t&,
                               const std::size_t, value_type&) {
  assert(false && "K-nearest neighbor in KD-tree: unimplemented features");
  return Nodes_t();
}

Nodes_t KDTree::KnearestSearch(ConfigurationIn_t, const RoadmapPtr_t&,
                               const std::size_t, value_type&) {
  assert(false && "K-nearest neighbor in KD-tree: unimplemented features");
  return Nodes_t();
}

NodeVector_t KDTree::withinBall(ConfigurationIn_t,
                                const ConnectedComponentPtr_t&, value_type) {
  assert(false && "withinBall in KD-tree: unimplemented features");
  return NodeVector_t();
}

void KDTree::search(value_type boxDistance, value_type& minDistance,
                    ConfigurationIn_t configuration,
                    const ConnectedComponentPtr_t& connectedComponent,
                    NodePtr_t& nearest, bool reverse) {
  if (boxDistance < minDistance * minDistance &&
      nodesMap_.count(connectedComponent) > 0) {
    // minDistance^2 because boxDistance is a squared distance
    if (infChild_ == NULL || supChild_ == NULL) {
      value_type distance = std::numeric_limits<value_type>::infinity();
      for (Nodes_t::iterator itNode = nodesMap_[connectedComponent].begin();
           itNode != nodesMap_[connectedComponent].end(); ++itNode) {
        if (reverse)
          distance = (*distance_)(configuration, (*itNode)->configuration());
        else
          distance = (*distance_)((*itNode)->configuration(), configuration);
        if (distance < minDistance) {
          minDistance = distance;
          nearest = (*itNode);
        }
      }
    } else {
      // find config to boxes distances
      value_type distanceToInfChild;
      value_type distanceToSupChild;
      if (boxDistance == 0.) {
        if (configuration[supChild_->splitDim_] >
            supChild_->lowerBounds_[supChild_->splitDim_]) {
          distanceToSupChild = 0.;
          distanceToInfChild = infChild_->distanceToBox(configuration);
        } else {
          distanceToInfChild = 0.;
          distanceToSupChild = supChild_->distanceToBox(configuration);
        }
      } else {
        distanceToInfChild = infChild_->distanceToBox(configuration);
        distanceToSupChild = supChild_->distanceToBox(configuration);
      }
      // search in the children
      if (distanceToInfChild < distanceToSupChild) {
        infChild_->search(boxDistance, minDistance, configuration,
                          connectedComponent, nearest);
        supChild_->search(
            boxDistance - distanceToInfChild * distanceToInfChild +
                distanceToSupChild * distanceToSupChild,
            minDistance, configuration, connectedComponent, nearest);
      } else {
        supChild_->search(boxDistance, minDistance, configuration,
                          connectedComponent, nearest);
        infChild_->search(
            boxDistance - distanceToSupChild * distanceToSupChild +
                distanceToInfChild * distanceToInfChild,
            minDistance, configuration, connectedComponent, nearest);
      }
    }
  }
}

void KDTree::merge(ConnectedComponentPtr_t cc1, ConnectedComponentPtr_t cc2) {
  nodesMap_[cc1].merge(nodesMap_[cc2]);
  NodesMap_t::iterator it = nodesMap_.find(cc2);
  nodesMap_.erase(it);
  if (infChild_ != NULL || supChild_ != NULL) {
    infChild_->merge(cc1, cc2);
    supChild_->merge(cc1, cc2);
  }
}
}  // namespace nearestNeighbor
}  // namespace core
}  // namespace hpp
