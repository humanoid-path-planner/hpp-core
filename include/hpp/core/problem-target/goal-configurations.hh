//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH
#define HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/problem-target.hh>

namespace hpp {
namespace core {
namespace problemTarget {
/// \addtogroup path_planning
/// \{

/// Goal configurations
///
/// This class defines a goal as a list of goal configurations.
class HPP_CORE_DLLAPI GoalConfigurations : public ProblemTarget {
 public:
  static GoalConfigurationsPtr_t create(const ProblemPtr_t& problem);

  /// Check if the problem target is well specified.
  void check(const RoadmapPtr_t& roadmap) const;

  /// Check if the initial configuration and one of the goal are
  /// in the same connected component.
  /// \note This test takes into account nodes of the roadmap that
  ///       are tagged as goal nodes, not the configurations stored
  ///       in this class. Be careful that those two sets are the
  ///       same.
  bool reached(const RoadmapPtr_t& roadmap) const;

  PathVectorPtr_t computePath(const RoadmapPtr_t& roadmap) const;
  /// Get goal configurations.
  const Configurations_t& configurations() const;
  /// Add goal configuration.
  void addConfiguration(const ConfigurationPtr_t& config);
  /// Reset the set of goal configurations
  void resetConfigurations();

 protected:
  /// Constructor
  GoalConfigurations(const ProblemPtr_t& problem) : ProblemTarget(problem) {}

 private:
  /// Shared pointer to goal configuration.
  Configurations_t configurations_;

};  // class GoalConfigurations
/// \}
}  // namespace problemTarget
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH
