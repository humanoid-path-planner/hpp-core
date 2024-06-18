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

#ifndef HPP_CORE_NODE_HH
#define HPP_CORE_NODE_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/serialization-fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup roadmap
/// \{

/// Node of a roadmap
///
/// Stores a configuration.
class HPP_CORE_DLLAPI Node {
 public:
  typedef std::list<EdgePtr_t> Edges_t;
  /// Constructor
  /// \param configuration configuration stored in the new node
  /// \note A new connected component is created. For consistency, the
  ///       new node is not registered in the connected component.
  Node(ConfigurationIn_t configuration);
  /// Constructor
  /// \param configuration configuration stored in the new node
  /// \param connectedComponent connected component the node belongs to.
  Node(ConfigurationIn_t configuration,
       ConnectedComponentPtr_t connectedComponent);
  void addOutEdge(EdgePtr_t edge);
  void addInEdge(EdgePtr_t edge);
  /// Store the connected component the node belongs to
  void connectedComponent(const ConnectedComponentPtr_t& cc);
  ConnectedComponentPtr_t connectedComponent() const;
  /// Access to outEdges
  const Edges_t& outEdges() const;
  /// Access to inEdges
  const Edges_t& inEdges() const;
  /// Check whether otherNode is an out-neighbor of this node.
  /// Node B is an out-neighbor of node A if node A has an outgoing edge
  /// going to B.
  bool isOutNeighbor(const NodePtr_t& n) const;
  /// Check whether otherNode is an in-neighbor of this node.
  /// Node B is an in-neighbor of node A if node A has an ingoing edge
  /// going to B.
  bool isInNeighbor(const NodePtr_t& n) const;

  const Configuration_t& configuration() const;
  /// Print node in a stream
  std::ostream& print(std::ostream& os) const;

  virtual ~Node() {};

 protected:
  Node() {}

 private:
  Configuration_t configuration_;
  Edges_t outEdges_;
  Edges_t inEdges_;
  ConnectedComponentPtr_t connectedComponent_;

  HPP_SERIALIZABLE();
};  // class Node
std::ostream& operator<<(std::ostream& os, const Node& n);
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_NODE_HH
