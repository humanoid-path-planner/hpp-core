//
// Copyright (c) 2020 CNRS
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

#ifndef HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH
#define HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH

#include <hpp/core/path-planner.hh>

namespace hpp {
namespace core {
namespace pathPlanner {
HPP_PREDEF_CLASS(BiRrtStar);
typedef shared_ptr<BiRrtStar> BiRrtStarPtr_t;

/// Bi-RRT* path planning algorithm
/// as described in http://www.golems.org/papers/AkgunIROS11-sampling.pdf
class HPP_CORE_DLLAPI BiRrtStar : public PathPlanner {
 public:
  typedef PathPlanner Parent_t;

  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  static BiRrtStarPtr_t create(const ProblemConstPtr_t& problem);
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  static BiRrtStarPtr_t createWithRoadmap(const ProblemConstPtr_t& problem,
                                          const RoadmapPtr_t& roadmap);

  /// Initialize the problem resolution
  ///  \li call parent implementation
  void startSolve();
  /// One step of the algorithm
  void oneStep();

 protected:
  /// Protected constructor
  /// \param problem the path planning problem
  BiRrtStar(const ProblemConstPtr_t& problem);
  /// Protected constructor
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  BiRrtStar(const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Store weak pointer to itself
  void init(const BiRrtStarWkPtr_t& weak);

 private:
  typedef std::map<NodePtr_t, EdgePtr_t> ParentMap_t;

  Configuration_t sample();

  /// Get cost to reach \c n
  value_type cost(NodePtr_t n);

  /// Set cost to reach \c n
  void cost(NodePtr_t n, value_type c);

  /// internal function to build path.
  /// It returns an empty path on failure.
  /// If \c validatePath is true, it returns only the valid part.
  /// If \c maxLength is positive, the returned path will be at most of this
  /// length.
  PathPtr_t buildPath(const Configuration_t& q0, const Configuration_t& q1,
                      value_type maxLength, bool validatePath);

  bool extend(NodePtr_t target, ParentMap_t& parentMap, Configuration_t& q);

  bool connect(NodePtr_t cc, ParentMap_t& parentMap, const Configuration_t& q);

  bool improve(const Configuration_t& q);

  value_type gamma_;
  /// Maximal path length with using function \ref extend.
  value_type extendMaxLength_;
  /// Minimal path length added to the roadmap.
  value_type minimalPathLength_;

  NodePtr_t roots_[2];

  /// store relation <child, parent> that brings to node \c roots_[i]
  std::vector<ParentMap_t> toRoot_;

  /// Weak pointer to itself
  BiRrtStarWkPtr_t weak_;
};  // class BiRrtStar
}  // namespace pathPlanner
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH
