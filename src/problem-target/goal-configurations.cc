// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/util/debug.hh>
#include <stdexcept>

#include "../astar.hh"

namespace hpp {
namespace core {
namespace problemTarget {
GoalConfigurationsPtr_t GoalConfigurations::create(
    const ProblemPtr_t& problem) {
  GoalConfigurations* gc = new GoalConfigurations(problem);
  GoalConfigurationsPtr_t shPtr(gc);
  gc->init(shPtr);
  return shPtr;
}

void GoalConfigurations::check(const RoadmapPtr_t& /*roadmap*/) const {}

bool GoalConfigurations::reached(const RoadmapPtr_t& roadmap) const {
  if (!roadmap->initNode()) return false;
  const ConnectedComponentPtr_t ccInit =
      roadmap->initNode()->connectedComponent();
  const NodeVector_t& goals = roadmap->goalNodes();
  for (NodeVector_t::const_iterator _goal = goals.begin(); _goal != goals.end();
       ++_goal) {
    if (ccInit->canReach((*_goal)->connectedComponent())) {
      return true;
    }
  }
  return false;
}

PathVectorPtr_t GoalConfigurations::computePath(
    const RoadmapPtr_t& roadmap) const {
  ProblemPtr_t problem(problem_.lock());
  assert(problem);
  Astar astar(roadmap, problem->distance());
  PathVectorPtr_t sol = PathVector::create(problem->robot()->configSize(),
                                           problem->robot()->numberDof());
  astar.solution(sol);
  // This happens when q_init == q_goal
  if (sol->numberPaths() == 0) {
    ConfigurationPtr_t q(roadmap->initNode()->configuration());
    sol->appendPath((*problem->steeringMethod())(*q, *q));
  }
  return sol;
}
// ======================================================================

const Configurations_t& GoalConfigurations::configurations() const {
  return configurations_;
}

// ======================================================================

void GoalConfigurations::addConfiguration(const ConfigurationPtr_t& config) {
  configurations_.push_back(config);
}

// ======================================================================

void GoalConfigurations::resetConfigurations() { configurations_.clear(); }

}  // namespace problemTarget
}  // namespace core
}  // namespace hpp
