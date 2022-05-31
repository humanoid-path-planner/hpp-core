//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana
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

#ifndef HPP_CORE_VISIBILITY_PRM_PLANNER_HH
#define HPP_CORE_VISIBILITY_PRM_PLANNER_HH

#include <hpp/core/path-planner.hh>
#include <tuple>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Generic implementation of visibility-PRM algorithm,
/// based on guard nodes (which cannot see each other) and
/// connection nodes between guards.
class HPP_CORE_DLLAPI VisibilityPrmPlanner : public PathPlanner {
 public:
  /// Return shared pointer to new object.
  static VisibilityPrmPlannerPtr_t createWithRoadmap(
      const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Return shared pointer to new object.
  static VisibilityPrmPlannerPtr_t create(const ProblemConstPtr_t& problem);
  /// One step of extension.
  virtual void oneStep();

 protected:
  /// Constructor
  VisibilityPrmPlanner(const ProblemConstPtr_t& problem,
                       const RoadmapPtr_t& roadmap);
  /// Constructor with roadmap
  VisibilityPrmPlanner(const ProblemConstPtr_t& problem);
  /// Store weak pointer to itself
  void init(const VisibilityPrmPlannerWkPtr_t& weak);

 private:
  typedef std::tuple<NodePtr_t, ConfigurationPtr_t, PathPtr_t> DelayedEdge_t;
  typedef std::vector<DelayedEdge_t> DelayedEdges_t;
  VisibilityPrmPlannerWkPtr_t weakPtr_;
  DelayedEdges_t delayedEdges_;
  std::map<NodePtr_t, bool> nodeStatus_;  // true for guard node

  /// Return true if the configuration is visible from the given
  /// connected component.
  bool visibleFromCC(const Configuration_t q, const ConnectedComponentPtr_t cc);

  /// Apply the problem constraints on a given configuration qTo by
  /// projecting it on the tangent space of qFrom.
  void applyConstraints(const Configuration_t& qFrom,
                        const Configuration_t& qTo, Configuration_t& qOut);

  bool constrApply_;  // True if applyConstraints has successed
};
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_VISIBILITY_PRM_PLANNER_HH
