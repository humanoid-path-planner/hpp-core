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

#include <../src/nearest-neighbor/basic.hh>
#include <algorithm>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/debug.hh>
#include <stdexcept>

namespace hpp {
namespace core {
using pinocchio::displayConfig;

RoadmapPtr_t Roadmap::create(const DistancePtr_t& distance,
                             const DevicePtr_t& robot) {
  Roadmap* ptr = new Roadmap(distance, robot);
  RoadmapPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

Roadmap::Roadmap(const DistancePtr_t& distance, const DevicePtr_t&)
    : distance_(distance),
      connectedComponents_(),
      nodes_(),
      edges_(),
      initNode_(),
      goalNodes_(),
      nearestNeighbor_(new nearestNeighbor::Basic(distance)) {}

Roadmap::~Roadmap() {
  clear();
  delete nearestNeighbor_;
}

const ConnectedComponents_t& Roadmap::connectedComponents() const {
  return connectedComponents_;
}

NearestNeighborPtr_t Roadmap::nearestNeighbor() { return nearestNeighbor_; }

void Roadmap::nearestNeighbor(NearestNeighborPtr_t nearestNeighbor) {
  if (nodes_.size() != 0) {
    throw std::runtime_error(
        "The roadmap must be empty before setting a new NearestNeighbor "
        "object.");
  }
  if (nearestNeighbor != NULL) {
    delete nearestNeighbor_;
    nearestNeighbor_ = nearestNeighbor;
  }
}

void Roadmap::clear() {
  connectedComponents_.clear();

  for (Nodes_t::iterator it = nodes_.begin(); it != nodes_.end(); ++it) {
    delete *it;
  }
  nodes_.clear();

  for (Edges_t::iterator it = edges_.begin(); it != edges_.end(); ++it) {
    delete *it;
  }
  edges_.clear();

  goalNodes_.clear();
  initNode_ = 0x0;
  nearestNeighbor_->clear();
}

NodePtr_t Roadmap::addNode(ConfigurationIn_t configuration) {
  value_type distance;
  if (nodes_.size() != 0) {
    NodePtr_t nearest = nearestNode(configuration, distance);
    if (nearest->configuration() == configuration) {
      return nearest;
    }
  }
  NodePtr_t node = createNode(configuration);
  hppDout(info, "Added node: " << displayConfig(configuration));
  push_node(node);
  // Node constructor creates a new connected component. This new
  // connected component needs to be added in the roadmap and the
  // new node needs to be registered in the connected component.
  addConnectedComponent(node);
  return node;
}

NodePtr_t Roadmap::addNode(ConfigurationIn_t configuration,
                           ConnectedComponentPtr_t connectedComponent) {
  assert(connectedComponent);
  value_type distance;
  if (nodes_.size() != 0) {
    NodePtr_t nearest =
        nearestNode(configuration, connectedComponent, distance);
    if (nearest->configuration() == configuration) {
      return nearest;
    }
  }
  NodePtr_t node = createNode(configuration);
  node->connectedComponent(connectedComponent);
  hppDout(info, "Added node: " << displayConfig(configuration));
  push_node(node);
  // The new node needs to be registered in the connected
  // component.
  connectedComponent->addNode(node);
  nearestNeighbor_->addNode(node);
  return node;
}

void Roadmap::addEdges(const NodePtr_t from, const NodePtr_t& to,
                       const PathPtr_t& path) {
  EdgePtr_t edge = new Edge(from, to, path);
  if (!from->isOutNeighbor(to)) from->addOutEdge(edge);
  if (!to->isInNeighbor(from)) to->addInEdge(edge);
  impl_addEdge(edge);
  edge = new Edge(to, from, path->reverse());
  if (!from->isInNeighbor(to)) from->addInEdge(edge);
  if (!to->isOutNeighbor(from)) to->addOutEdge(edge);
  impl_addEdge(edge);
}

NodePtr_t Roadmap::addNodeAndEdges(const NodePtr_t from, ConfigurationIn_t to,
                                   const PathPtr_t path) {
  NodePtr_t nodeTo = addNode(to, from->connectedComponent());
  addEdges(from, nodeTo, path);
  return nodeTo;
}

NodePtr_t Roadmap::addNodeAndEdge(const NodePtr_t from, ConfigurationIn_t to,
                                  const PathPtr_t path) {
  NodePtr_t nodeTo = addNode(to, from->connectedComponent());
  addEdge(from, nodeTo, path);
  return nodeTo;
}

NodePtr_t Roadmap::addNodeAndEdge(ConfigurationIn_t from, const NodePtr_t to,
                                  const PathPtr_t path) {
  NodePtr_t nodeFrom = addNode(from, to->connectedComponent());
  addEdge(nodeFrom, to, path);
  return nodeFrom;
}

void Roadmap::merge(const RoadmapPtr_t& other) {
  // Map nodes of other roadmap with nodes of this one
  std::map<core::NodePtr_t, core::NodePtr_t> cNode;
  for (const core::NodePtr_t& node : other->nodes()) {
    cNode[node] = this->addNode(node->configuration());
  }
  for (const core::EdgePtr_t& edge : other->edges()) {
    if (edge->path()->length() == 0)
      assert(edge->from() == edge->to());
    else
      this->addEdges(cNode[edge->from()], cNode[edge->to()], edge->path());
  }
}

void Roadmap::insertPathVector(const PathVectorPtr_t& path, bool backAndForth) {
  if (path->constraints()) {
    throw std::logic_error(
        "Cannot insert a path vector with constraints"
        " in a roadmap.");
  }
  Configuration_t q_init(path->initial());
  NodePtr_t n(addNode(q_init));
  for (std::size_t i = 0; i < path->numberPaths(); ++i) {
    PathPtr_t p(path->pathAtRank(i));
    if (backAndForth) {
      n = addNodeAndEdges(n, p->end(), p);
    } else {
      n = addNodeAndEdge(n, p->end(), p);
    }
  }
}

NodePtr_t Roadmap::nearestNode(ConfigurationIn_t configuration,
                               value_type& minDistance, bool reverse) {
  NodePtr_t closest = 0x0;
  minDistance = std::numeric_limits<value_type>::infinity();
  for (ConnectedComponents_t::const_iterator itcc =
           connectedComponents_.begin();
       itcc != connectedComponents_.end(); ++itcc) {
    value_type distance;
    NodePtr_t node;
    node = nearestNeighbor_->search(configuration, *itcc, distance, reverse);
    if (distance < minDistance) {
      minDistance = distance;
      closest = node;
    }
  }
  return closest;
}

NodePtr_t Roadmap::nearestNode(
    ConfigurationIn_t configuration,
    const ConnectedComponentPtr_t& connectedComponent, value_type& minDistance,
    bool reverse) {
  assert(connectedComponent);
  assert(connectedComponent->nodes().size() != 0);
  NodePtr_t closest = nearestNeighbor_->search(
      configuration, connectedComponent, minDistance, reverse);
  return closest;
}

Nodes_t Roadmap::nearestNodes(ConfigurationIn_t configuration, size_type k) {
  value_type d;
  return nearestNeighbor_->KnearestSearch(configuration, weak_.lock(), k, d);
}

Nodes_t Roadmap::nearestNodes(ConfigurationIn_t configuration,
                              const ConnectedComponentPtr_t& connectedComponent,
                              size_type k) {
  value_type d;
  return nearestNeighbor_->KnearestSearch(configuration, connectedComponent, k,
                                          d);
}

NodeVector_t Roadmap::nodesWithinBall(ConfigurationIn_t q,
                                      const ConnectedComponentPtr_t& cc,
                                      value_type maxD) {
  return nearestNeighbor_->withinBall(q, cc, maxD);
}

NodePtr_t Roadmap::addGoalNode(ConfigurationIn_t config) {
  NodePtr_t node = addNode(config);
  goalNodes_.push_back(node);
  return node;
}

const DistancePtr_t& Roadmap::distance() const { return distance_; }

EdgePtr_t Roadmap::addEdge(const NodePtr_t& n1, const NodePtr_t& n2,
                           const PathPtr_t& path) {
  EdgePtr_t edge = new Edge(n1, n2, path);
  if (!n1->isOutNeighbor(n2)) n1->addOutEdge(edge);
  if (!n2->isInNeighbor(n1)) n2->addInEdge(edge);
  impl_addEdge(edge);
  return edge;
}

void Roadmap::addConnectedComponent(const NodePtr_t& node) {
  connectedComponents_.insert(node->connectedComponent());
  node->connectedComponent()->addNode(node);
  nearestNeighbor_->addNode(node);
}

NodePtr_t Roadmap::createNode(ConfigurationIn_t configuration) const {
  return NodePtr_t(new Node(configuration));
}

void Roadmap::init(RoadmapWkPtr_t weak) { weak_ = weak; }

void Roadmap::connect(const ConnectedComponentPtr_t& cc1,
                      const ConnectedComponentPtr_t& cc2) {
  if (cc1->canReach(cc2)) return;
  ConnectedComponent::RawPtrs_t cc2Tocc1;
  if (cc2->canReach(cc1, cc2Tocc1)) {
    merge(cc1, cc2Tocc1);
  } else {
    cc1->reachableTo_.insert(cc2.get());
    cc2->reachableFrom_.insert(cc1.get());
  }
}

void Roadmap::merge(const ConnectedComponentPtr_t& cc1,
                    ConnectedComponent::RawPtrs_t& ccs) {
  for (ConnectedComponent::RawPtrs_t::iterator itcc = ccs.begin();
       itcc != ccs.end(); ++itcc) {
    if (*itcc != cc1.get()) {
      cc1->merge((*itcc)->self());
#ifndef NDEBUG
      std::size_t nb =
#endif
          connectedComponents_.erase((*itcc)->self());
      assert(nb == 1);
    }
  }
}

void Roadmap::impl_addEdge(const EdgePtr_t& edge) {
  edges_.push_back(edge);

  ConnectedComponentPtr_t cc1 = edge->from()->connectedComponent();
  ConnectedComponentPtr_t cc2 = edge->to()->connectedComponent();

  connect(cc1, cc2);
}

bool Roadmap::pathExists() const {
  const ConnectedComponentPtr_t ccInit = initNode()->connectedComponent();
  for (NodeVector_t::const_iterator itGoal = goalNodes_.begin();
       itGoal != goalNodes_.end(); ++itGoal) {
    if (ccInit->canReach((*itGoal)->connectedComponent())) {
      return true;
    }
  }
  return false;
}

std::ostream& Roadmap::print(std::ostream& os) const {
  // Enumerate nodes and connected components
  std::map<NodePtr_t, size_type> nodeId;
  std::map<ConnectedComponent::RawPtr_t, size_type> ccId;
  std::map<ConnectedComponent::RawPtr_t, size_type> sccId;

  size_type count = 0;
  for (Nodes_t::const_iterator it = nodes().begin(); it != nodes().end();
       ++it) {
    nodeId[*it] = count;
    ++count;
  }

  count = 0;
  for (ConnectedComponents_t::const_iterator it = connectedComponents().begin();
       it != connectedComponents().end(); ++it) {
    ccId[it->get()] = count;
    ++count;
  }

  // Display list of nodes
  os << "-----------------------------------------------------------------"
     << std::endl;
  os << "Roadmap" << std::endl;
  os << "-----------------------------------------------------------------"
     << std::endl;
  os << "-----------------------------------------------------------------"
     << std::endl;
  os << "Nodes" << std::endl;
  os << "-----------------------------------------------------------------"
     << std::endl;
  for (Nodes_t::const_iterator it = nodes().begin(); it != nodes().end();
       ++it) {
    const NodePtr_t node = *it;
    os << "Node " << nodeId[node] << ": " << *node << std::endl;
  }
  os << "-----------------------------------------------------------------"
     << std::endl;
  os << "Edges" << std::endl;
  os << "-----------------------------------------------------------------"
     << std::endl;
  for (Edges_t::const_iterator it = edges().begin(); it != edges().end();
       ++it) {
    const EdgePtr_t edge = *it;
    os << "Edge: " << nodeId[edge->from()] << " -> " << nodeId[edge->to()]
       << std::endl;
  }
  os << "-----------------------------------------------------------------"
     << std::endl;
  os << "Connected components" << std::endl;
  os << "-----------------------------------------------------------------"
     << std::endl;
  for (ConnectedComponents_t::const_iterator it = connectedComponents().begin();
       it != connectedComponents().end(); ++it) {
    const ConnectedComponentPtr_t cc = *it;
    os << "Connected component " << ccId[cc.get()] << std::endl;
    os << "Nodes : ";
    for (NodeVector_t::const_iterator itNode = cc->nodes().begin();
         itNode != cc->nodes().end(); ++itNode) {
      os << nodeId[*itNode] << ", ";
    }
    os << std::endl;
    os << "Reachable to :";
    for (ConnectedComponent::RawPtrs_t::const_iterator itTo =
             cc->reachableTo().begin();
         itTo != cc->reachableTo().end(); ++itTo) {
      os << ccId[*itTo] << ", ";
    }
    os << std::endl;
    os << "Reachable from :";
    for (ConnectedComponent::RawPtrs_t::const_iterator itFrom =
             cc->reachableFrom().begin();
         itFrom != cc->reachableFrom().end(); ++itFrom) {
      os << ccId[*itFrom] << ", ";
    }
    os << std::endl;
  }
  os << std::endl;
  os << "----------------" << std::endl;

  return os;
}

std::ostream& operator<<(std::ostream& os, const hpp::core::Roadmap& r) {
  return r.print(os);
}
}  //   namespace core
}  // namespace hpp
