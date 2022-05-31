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

#ifndef HPP_CORE_BIRRT_PLANNER_HH
#define HPP_CORE_BIRRT_PLANNER_HH

#include <hpp/core/path-planner.hh>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Implementation of directional bi-RRT algorithm
/// maintaining only two connected components for
/// respectively the start and goal configurations
class HPP_CORE_DLLAPI BiRRTPlanner : public PathPlanner {
 public:
  /// Return shared pointer to new object.
  static BiRRTPlannerPtr_t createWithRoadmap(const ProblemConstPtr_t& problem,
                                             const RoadmapPtr_t& roadmap);
  /// Return shared pointer to new object.
  static BiRRTPlannerPtr_t create(const ProblemConstPtr_t& problem);
  /// One step of extension.
  virtual void startSolve();
  /// One step of extension.
  virtual void oneStep();

 protected:
  /// Constructor
  BiRRTPlanner(const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
  /// Constructor with roadmap
  BiRRTPlanner(const ProblemConstPtr_t& problem);
  /// Store weak pointer to itself
  void init(const BiRRTPlannerWkPtr_t& weak);
  PathPtr_t extendInternal(const SteeringMethodPtr_t& sm,
                           Configuration_t& qProj_, const NodePtr_t& near,
                           const Configuration_t& target, bool reverse = false);

  ConfigurationShooterPtr_t configurationShooter_;
  ConnectedComponentPtr_t startComponent_;
  std::vector<ConnectedComponentPtr_t> endComponents_;

 private:
  mutable Configuration_t qProj_;
  BiRRTPlannerWkPtr_t weakPtr_;
};
/// \}
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_BIRRT_PLANNER_HH
