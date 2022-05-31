// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#include "../src/nearest-neighbor/basic.hh"

#include <hpp/core/connected-component.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/roadmap.hh>
#include <limits>
#include <queue>

namespace hpp {
namespace core {
namespace nearestNeighbor {
namespace {
typedef std::pair<value_type, NodePtr_t> DistAndNode_t;
struct DistAndNodeComp_t {
  bool operator()(const DistAndNode_t& r, const DistAndNode_t& l) {
    return r.first < l.first;
  }
};
typedef std::priority_queue<DistAndNode_t, std::vector<DistAndNode_t>,
                            DistAndNodeComp_t>
    Queue_t;
}  // namespace

NodePtr_t Basic::search(const Configuration_t& configuration,
                        const ConnectedComponentPtr_t& connectedComponent,
                        value_type& distance, bool reverse) {
  NodePtr_t result = NULL;
  distance = std::numeric_limits<value_type>::infinity();
  const Distance& dist = *distance_;
  value_type d;
  for (NodeVector_t::const_iterator itNode =
           connectedComponent->nodes().begin();
       itNode != connectedComponent->nodes().end(); ++itNode) {
    if (reverse)
      d = dist(configuration, *(*itNode)->configuration());
    else
      d = dist(*(*itNode)->configuration(), configuration);
    if (d < distance) {
      distance = d;
      result = *itNode;
    }
  }
  assert(result);
  return result;
}

NodePtr_t Basic::search(const NodePtr_t& node,
                        const ConnectedComponentPtr_t& connectedComponent,
                        value_type& distance) {
  NodePtr_t result = NULL;
  distance = std::numeric_limits<value_type>::infinity();
  const Distance& dist = *distance_;
  for (NodeVector_t::const_iterator itNode =
           connectedComponent->nodes().begin();
       itNode != connectedComponent->nodes().end(); ++itNode) {
    value_type d = dist(*itNode, node);
    if (d < distance) {
      distance = d;
      result = *itNode;
    }
  }
  assert(result);
  return result;
}

Nodes_t Basic::KnearestSearch(const Configuration_t& q,
                              const ConnectedComponentPtr_t& connectedComponent,
                              const std::size_t K, value_type& distance) {
  Queue_t ns;
  distance = std::numeric_limits<value_type>::infinity();
  const Distance& dist = *distance_;
  for (NodeVector_t::const_iterator itNode =
           connectedComponent->nodes().begin();
       itNode != connectedComponent->nodes().end(); ++itNode) {
    value_type d = dist(*(*itNode)->configuration(), q);
    if (ns.size() < K)
      ns.push(DistAndNode_t(d, (*itNode)));
    else if (ns.top().first > d) {
      ns.pop();
      ns.push(DistAndNode_t(d, (*itNode)));
    }
  }
  Nodes_t nodes;
  if (ns.size() > 0) distance = ns.top().first;
  while (ns.size() > 0) {
    nodes.push_front(ns.top().second);
    ns.pop();
  }
  return nodes;
}

Nodes_t Basic::KnearestSearch(const NodePtr_t& node,
                              const ConnectedComponentPtr_t& connectedComponent,
                              const std::size_t K, value_type& distance) {
  Queue_t ns;
  distance = std::numeric_limits<value_type>::infinity();
  const Distance& dist = *distance_;
  for (NodeVector_t::const_iterator itNode =
           connectedComponent->nodes().begin();
       itNode != connectedComponent->nodes().end(); ++itNode) {
    value_type d = dist(*itNode, node);
    if (ns.size() < K)
      ns.push(DistAndNode_t(d, (*itNode)));
    else if (ns.top().first > d) {
      ns.pop();
      ns.push(DistAndNode_t(d, (*itNode)));
    }
  }
  Nodes_t nodes;
  if (ns.size() > 0) distance = ns.top().first;
  while (ns.size() > 0) {
    nodes.push_front(ns.top().second);
    ns.pop();
  }
  return nodes;
}

Nodes_t Basic::KnearestSearch(const Configuration_t& q,
                              const RoadmapPtr_t& roadmap, const std::size_t K,
                              value_type& distance) {
  Queue_t ns;
  distance = std::numeric_limits<value_type>::infinity();
  const Distance& dist = *distance_;
  for (Nodes_t::const_iterator itNode = roadmap->nodes().begin();
       itNode != roadmap->nodes().end(); ++itNode) {
    value_type d = dist(*(*itNode)->configuration(), q);
    if (ns.size() < K)
      ns.push(DistAndNode_t(d, (*itNode)));
    else if (ns.top().first > d) {
      ns.pop();
      ns.push(DistAndNode_t(d, (*itNode)));
    }
  }
  Nodes_t nodes;
  if (ns.size() > 0) distance = ns.top().first;
  while (ns.size() > 0) {
    nodes.push_front(ns.top().second);
    ns.pop();
  }
  return nodes;
}

NodeVector_t Basic::withinBall(const Configuration_t& q,
                               const ConnectedComponentPtr_t& cc,
                               value_type maxDistance) {
  const Distance& dist = *distance_;
  NodeVector_t nodes;
  for (NodeVector_t::const_iterator itNode = cc->nodes().begin();
       itNode != cc->nodes().end(); ++itNode) {
    NodePtr_t n = *itNode;
    if (dist(*n->configuration(), q) < maxDistance) nodes.push_back(n);
  }
  return nodes;
}
}  // namespace nearestNeighbor
}  // namespace core
}  // namespace hpp
