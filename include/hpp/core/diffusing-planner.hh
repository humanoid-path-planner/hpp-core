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

#ifndef HPP_CORE_DIFFUSING_PLANNER_HH
#define HPP_CORE_DIFFUSING_PLANNER_HH

#include <hpp/core/path-planner.hh>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Generic implementation of RRT algorithm
class HPP_CORE_DLLAPI DiffusingPlanner : public PathPlanner {
 public:
  typedef PathPlanner Parent_t;
  /// Return shared pointer to new object.
  static DiffusingPlannerPtr_t createWithRoadmap(
      const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Return shared pointer to new object.
  static DiffusingPlannerPtr_t create(const ProblemConstPtr_t& problem);
  /// Initialize the problem resolution
  /// Call parent implementation and check that goal is defined as
  /// a set of configurations.
  virtual void startSolve();
  /// One step of extension.
  virtual void oneStep();
  /// Set configuration shooter.
  void configurationShooter(const ConfigurationShooterPtr_t& shooter);

 protected:
  /// Constructor
  DiffusingPlanner(const ProblemConstPtr_t& problem,
                   const RoadmapPtr_t& roadmap);
  /// Constructor with roadmap
  DiffusingPlanner(const ProblemConstPtr_t& problem);
  /// Store weak pointer to itself
  void init(const DiffusingPlannerWkPtr_t& weak);
  /// Extend a node in the direction of a configuration
  /// \param near node in the roadmap,
  /// \param target target configuration
  virtual PathPtr_t extend(const NodePtr_t& near,
                           ConfigurationIn_t target);

 private:
  ConfigurationShooterPtr_t configurationShooter_;
  mutable Configuration_t qProj_;
  DiffusingPlannerWkPtr_t weakPtr_;
};
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_DIFFUSING_PLANNER_HH
