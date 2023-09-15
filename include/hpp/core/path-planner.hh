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

#ifndef HPP_CORE_PATH_PLANNER_HH
#define HPP_CORE_PATH_PLANNER_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Path planner
///
/// Algorithm that computes a path between an initial configuration and a
/// set of goal configurations.
class HPP_CORE_DLLAPI PathPlanner {
 public:
  virtual ~PathPlanner();

  /// Get roadmap
  virtual const RoadmapPtr_t& roadmap() const;
  /// Get problem
  ProblemConstPtr_t problem() const;
  /// Initialize the problem resolution
  ///  \li Set initial and and goal nodes,
  ///  \li check problem consistency
  virtual void startSolve();
  /// Solve
  ///
  /// Call methods
  /// \li startSolve,
  /// \li oneStep until a solution is found,
  /// \li finishSolve.
  /// Users can implement themselves the loop to avoid being trapped
  /// in an infinite loop when no solution is found.
  virtual PathVectorPtr_t solve();
  /// Try to connect initial and goal configurations to existing roadmap
  virtual void tryConnectInitAndGoals();

  /// User implementation of one step of resolution
  virtual void oneStep() = 0;
  /// Post processing of the resulting path
  virtual PathVectorPtr_t finishSolve(const PathVectorPtr_t& path);
  /// Interrupt path planning
  void interrupt();
  /// Set maximal number of iterations
  void maxIterations(const unsigned long int& n);
  /// Get maximal number of iterations
  unsigned long int maxIterations() const {
    return maxIterations_;
  }
  /// set time out (in seconds)
  void timeOut(const double& timeOut);
  /// Get time out
  double timeOut() const {
    return timeOut_;
  }
  /// Make the resolution stop when the problem is solved.
  /// If set to \c false, the algorithm stops when \ref maxIterations
  /// or \ref timeOut are reached and it is a success if the
  /// \ref Problem::target is achieved.
  void stopWhenProblemIsSolved(bool enable);

  /// Find a path in the roadmap and transform it in trajectory
  PathVectorPtr_t computePath() const;

 protected:
  /// Constructor
  ///
  /// Create a new roadmap
  PathPlanner(const ProblemConstPtr_t& problem);
  /// Constructor
  ///
  /// Store a given roadmap.
  PathPlanner(const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Store weak pointer to itself
  void init(const PathPlannerWkPtr_t& weak);

 private:
  /// Reference to the problem
  const ProblemConstWkPtr_t problem_;
  /// Pointer to the roadmap.
  const RoadmapPtr_t roadmap_;
  bool interrupt_;
  /// Maximal number of iterations to solve a problem
  /// reaching this bound raises an exception.
  unsigned long int maxIterations_;
  /// Time out (in seconds) before interrupting the planning
  double timeOut_;
  /// \copydoc PathPlanner::stopWhenProblemIsSolved
  bool stopWhenProblemIsSolved_;

  /// Store weak pointer to itself
  PathPlannerWkPtr_t weakPtr_;
};  // class PathPlanner
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_PLANNER_HH
