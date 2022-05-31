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

#ifndef HPP_CORE_CONNECTED_COMPONENT_HH
#define HPP_CORE_CONNECTED_COMPONENT_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/node.hh>
#include <hpp/util/serialization-fwd.hh>

namespace hpp {
namespace core {
/// Connected component
///
/// Set of nodes reachable from one another.
class HPP_CORE_DLLAPI ConnectedComponent {
 public:
  typedef ConnectedComponent* RawPtr_t;
  typedef std::set<RawPtr_t> RawPtrs_t;

  // variable for ranking connected components
  static unsigned int globalFinishTime_;
  static ConnectedComponentPtr_t create() {
    ConnectedComponent* ptr = new ConnectedComponent();
    ConnectedComponentPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }
  /// Merge two connected components.
  ///
  /// \param other connected component to merge into this one.
  /// \note other will be empty after calling this method.
  virtual void merge(const ConnectedComponentPtr_t& other);

  virtual ~ConnectedComponent() {}

  /// Add node in connected component
  /// \param node node to add.
  virtual void addNode(const NodePtr_t& node) { nodes_.push_back(node); }
  /// Access to the nodes
  const NodeVector_t& nodes() const { return nodes_; }

  /// \name Reachability
  /// \{

  /// Whether this connected component can reach cc
  /// \param cc a connected component
  bool canReach(const ConnectedComponentPtr_t& cc);

  /// Whether this connected component can reach cc
  /// \param cc a connected component
  /// \retval cc2Tocc1 list of connected components between cc2 and cc1
  ///         that should be merged.
  bool canReach(const ConnectedComponentPtr_t& cc, RawPtrs_t& cc2Tocc1);

  // Get connected components reachable from this
  const RawPtrs_t& reachableTo() const { return reachableTo_; }

  // Get connected components that can reach this
  const RawPtrs_t& reachableFrom() const { return reachableFrom_; }
  /// \}

  ConnectedComponentPtr_t self() { return weak_.lock(); }

 protected:
  /// Constructor
  ConnectedComponent() : nodes_(), explored_(false), weak_() {
    nodes_.reserve(1000);
  }
  void init(const ConnectedComponentPtr_t& shPtr) { weak_ = shPtr; }

 private:
  static void clean(RawPtrs_t& set);

  NodeVector_t nodes_;
  // List of CCs from which this connected component can be reached
  RawPtrs_t reachableFrom_;
  // List of CCs that can be reached from this connected component
  RawPtrs_t reachableTo_;
  // status variable to indicate whether or not CC has been visited
  mutable bool explored_;
  ConnectedComponentWkPtr_t weak_;
  friend class Roadmap;

  HPP_SERIALIZABLE();
};  // class ConnectedComponent
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONNECTED_COMPONENT_HH
