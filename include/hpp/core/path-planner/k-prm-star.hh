//
// Copyright (c) 2018 CNRS
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

#ifndef HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH
#define HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH

#include <hpp/core/path-planner.hh>

namespace hpp {
namespace core {
namespace pathPlanner {

/// \addtogroup path_planning
/// \{

/// k-PRM* path planning algorithm
/// as described in https://arxiv.org/pdf/1105.1186.pdf.
class HPP_CORE_DLLAPI kPrmStar : public PathPlanner {
 public:
  /// Computation step of the algorithm
  enum STATE {
    BUILD_ROADMAP,
    LINK_NODES,
    CONNECT_INIT_GOAL,
    FAILURE
  };  // enum STATE
  /// Constant kPRM = 2 e
  static const double kPRM;
  typedef PathPlanner Parent_t;
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  static kPrmStarPtr_t create(const ProblemConstPtr_t& problem);
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  static kPrmStarPtr_t createWithRoadmap(const ProblemConstPtr_t& problem,
                                         const RoadmapPtr_t& roadmap);
  /// Initialize the problem resolution
  ///  \li call parent implementation
  ///  \li get number nodes in problem parameter map
  virtual void startSolve();
  /// Does nothing.
  /// This step is performed by the main algorithm
  virtual void tryConnectInitAndGoals();
  /// One step of the algorithm
  virtual void oneStep();
  /// get the computationnal state of the algorithm
  STATE getComputationState() const;

 protected:
  /// Protected constructor
  /// \param problem the path planning problem
  kPrmStar(const ProblemConstPtr_t& problem);
  /// Protected constructor
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  kPrmStar(const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Store weak pointer to itself
  void init(const kPrmStarWkPtr_t& weak);

 private:
  STATE state_;
  /// Generate random free configurations 10 by 10
  void generateRandomConfig();
  /// Link each node with closest neighbors
  void linkNodes();
  /// Connect initial and goal configurations to roadmap
  void connectInitAndGoal();
  /// Connect node to k closest neighbors in the roadmap
  /// \param node node to connect to nearest neighbors,
  /// \return whether iterator on neighbors reached the end.
  bool connectNodeToClosestNeighbors(const NodePtr_t& node);
  /// Number of nodes to create
  std::size_t numberNodes_;
  /// Iterator on nodes
  Nodes_t::const_iterator linkingNodeIt_;
  /// Iterator on neighbors
  Nodes_t::iterator itNeighbor_;
  /// Number of closest neighbors to connect to each node
  size_type numberNeighbors_;
  /// List of neighbors of current node
  Nodes_t neighbors_;
  /// whether iterator reached last neighbor
  bool reachedLastNeighbor_;
  /// Weak pointer to itself
  kPrmStarWkPtr_t weak_;
};  // class kPrmStar

/// \}

}  // namespace pathPlanner
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH
