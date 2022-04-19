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

#ifndef HPP_CORE_PLAN_AND_OPTIMIZE_HH
#define HPP_CORE_PLAN_AND_OPTIMIZE_HH

#include <hpp/core/config.hh>
#include <hpp/core/path-planner.hh>
#include <vector>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Path planner and optimizer
///
/// Plans a path and iteratively applies a series of optimizer on
/// the result.
class HPP_CORE_DLLAPI PlanAndOptimize : public PathPlanner {
 public:
  /// Return shared pointer to new object.
  static PlanAndOptimizePtr_t create(const PathPlannerPtr_t& pathPlanner);
  /// Call internal path planner implementation
  virtual void startSolve();
  /// One iteration of path planning or path optimization
  virtual void oneStep();
  /// Optimize planned path
  virtual PathVectorPtr_t finishSolve(const PathVectorPtr_t& path);
  void addPathOptimizer(const PathOptimizerPtr_t& optimizer);

 protected:
  PlanAndOptimize(const PathPlannerPtr_t& pathPlanner);

 private:
  typedef std::vector<PathOptimizerPtr_t> Optimizers_t;
  const PathPlannerPtr_t pathPlanner_;
  Optimizers_t optimizers_;
};  // class PlanAndOptimize
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PLAN_AND_OPTIMIZE_HH
