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

#ifndef HPP_CORE_ASTAR_HH
#define HPP_CORE_ASTAR_HH

#include <hpp/core/config.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-vector.hh>
#include <limits>

namespace hpp {
namespace core {
class HPP_CORE_LOCAL Astar {
  typedef std::list<NodePtr_t> Nodes_t;
  typedef std::list<EdgePtr_t> Edges_t;
  typedef std::map<NodePtr_t, EdgePtr_t> Parent_t;
  Nodes_t closed_;
  Nodes_t open_;
  std::map<NodePtr_t, value_type> costFromStart_;
  std::map<NodePtr_t, value_type> estimatedCostToGoal_;
  Parent_t parent_;
  RoadmapPtr_t roadmap_;
  DistancePtr_t distance_;

 public:
  Astar(const RoadmapPtr_t& roadmap, const DistancePtr_t distance)
      : roadmap_(roadmap), distance_(distance) {}

  void solution(PathVectorPtr_t sol) {
    NodePtr_t node = findPath();
    Edges_t edges;

    while (node) {
      Parent_t::const_iterator itEdge = parent_.find(node);
      if (itEdge != parent_.end()) {
        EdgePtr_t edge = itEdge->second;
        edges.push_front(edge);
        node = edge->from();
      } else
        node = NodePtr_t(0x0);
    }
    for (Edges_t::const_iterator itEdge = edges.begin(); itEdge != edges.end();
         ++itEdge) {
      const PathPtr_t& path((*itEdge)->path());
      sol->appendPath(path);
    }
  }

 private:
  struct SortFunctor {
    std::map<NodePtr_t, value_type>& cost_;
    SortFunctor(std::map<NodePtr_t, value_type>& cost) : cost_(cost) {}
    bool operator()(const NodePtr_t& n1, const NodePtr_t& n2) {
      return cost_[n1] < cost_[n2];
    }
  };  // struc SortFunctor

  bool isGoal(const NodePtr_t node) {
    for (NodeVector_t::const_iterator itGoal = roadmap_->goalNodes().begin();
         itGoal != roadmap_->goalNodes().end(); ++itGoal) {
      if (*itGoal == node) {
        return true;
      }
    }
    return false;
  }

  NodePtr_t findPath() {
    closed_.clear();
    open_.clear();
    parent_.clear();
    estimatedCostToGoal_.clear();
    costFromStart_.clear();

    open_.push_back(roadmap_->initNode());
    while (!open_.empty()) {
      open_.sort(SortFunctor(estimatedCostToGoal_));
      Nodes_t::iterator itv = open_.begin();
      NodePtr_t current(*itv);
      if (isGoal(current)) {
        return current;
      }
      open_.erase(itv);
      closed_.push_back(current);
      for (Edges_t::const_iterator itEdge = current->outEdges().begin();
           itEdge != current->outEdges().end(); ++itEdge) {
        value_type transitionCost = edgeCost(*itEdge);
        NodePtr_t child((*itEdge)->to());
        if (std::find(closed_.begin(), closed_.end(), child) == closed_.end()) {
          // node is not in closed set
          value_type tmpCost = costFromStart_[current] + transitionCost;
          bool childNotInOpenSet =
              (std::find(open_.begin(), open_.end(), child) == open_.end());
          if ((childNotInOpenSet) || (tmpCost < costFromStart_[child])) {
            parent_[child] = *itEdge;
            costFromStart_[child] = tmpCost;
            estimatedCostToGoal_[child] =
                costFromStart_[child] + heuristic(child);
            if (childNotInOpenSet) open_.push_back(child);
          }
        }
      }
    }
    throw std::runtime_error("A* failed to find a solution to the goal.");
  }

  value_type heuristic(const NodePtr_t node) const {
    Configuration_t config = node->configuration();
    value_type res = std::numeric_limits<value_type>::infinity();
    for (NodeVector_t::const_iterator itGoal = roadmap_->goalNodes().begin();
         itGoal != roadmap_->goalNodes().end(); ++itGoal) {
      Configuration_t goal = (*itGoal)->configuration();
      value_type dist = (*distance_)(config, goal);
      if (dist < res) {
        res = dist;
      }
    }
    return res;
  }

  value_type edgeCost(const EdgePtr_t& edge) { return edge->path()->length(); }
};  // class Astar
}  //   namespace core
}  // namespace hpp

#endif  // HPP_CORE_ASTAR_HH
