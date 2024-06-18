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

#ifndef HPP_CORE_PROBLEM_TARGET_HH
#define HPP_CORE_PROBLEM_TARGET_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Problem target
///
/// This abstract class defines the goal to be reached by a planning
/// algorithm.
class HPP_CORE_DLLAPI ProblemTarget {
 public:
  virtual ~ProblemTarget() {};

  /// Check if the problem target is well specified.
  virtual void check(const RoadmapPtr_t& roadmap) const = 0;

  /// Check whether the problem is solved.
  virtual bool reached(const RoadmapPtr_t& roadmap) const = 0;

  /// Returns the solution path found.
  /// Should be called when reached() returns true.
  virtual PathVectorPtr_t computePath(const RoadmapPtr_t& roadmap) const = 0;

  /// Set the problem
  void problem(const ProblemPtr_t& problem) { problem_ = problem; }

 protected:
  /// Constructor
  ProblemTarget(const ProblemPtr_t& problem) : problem_(problem) {}

  /// Store weak pointer to itself
  void init(const ProblemTargetWkPtr_t& weak) { weakPtr_ = weak; }

  /// Reference to the planner for access to problem and roadmap
  ProblemWkPtr_t problem_;

  /// Store weak pointer to itself
  ProblemTargetWkPtr_t weakPtr_;

 private:
};  // class ProblemTarget
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_PROBLEM_TARGET_HH
