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

#include <hpp/core/config-projector.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/node.hh>
#include <hpp/core/problem-target/task-target.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <stdexcept>

#include "../astar.hh"

namespace hpp {
namespace core {
namespace problemTarget {
TaskTargetPtr_t TaskTarget::create(const ProblemPtr_t& problem) {
  TaskTarget* tt = new TaskTarget(problem);
  TaskTargetPtr_t shPtr(tt);
  tt->init(shPtr);
  return shPtr;
}

void TaskTarget::check(const RoadmapPtr_t&) const {
  if (!constraints_) {
    std::string msg("No constraints: task not specified.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }
}

bool TaskTarget::reached(const RoadmapPtr_t& roadmap) const {
  assert(roadmap->goalNodes().empty());
  if (!roadmap->initNode()) return false;
  const ConnectedComponentPtr_t ccInit =
      roadmap->initNode()->connectedComponent();  // TODO
  bool res(false);
  for (auto node : ccInit->nodes()) {
    if ((*constraints_).isSatisfied(*(node->configuration()))) {
      roadmap->addGoalNode(
          node->configuration());  // temporarily add goal node to compute path
      res = true;
    }
  }
  return res;
}

NumericalConstraints_t TaskTarget::constraints() const {
  if ((!constraints_) || (!constraints_->configProjector())) {
    return NumericalConstraints_t();
  }
  return constraints_->configProjector()->numericalConstraints();
}

PathVectorPtr_t TaskTarget::computePath(const RoadmapPtr_t& roadmap) const {
  ProblemPtr_t problem(problem_.lock());
  assert(problem);
  Astar astar(roadmap, problem->distance());
  PathVectorPtr_t sol = PathVector::create(problem->robot()->configSize(),
                                           problem->robot()->numberDof());
  astar.solution(sol);
  // This happens when q_init already satisfies the task.
  if (sol->numberPaths() == 0) {
    ConfigurationPtr_t q(roadmap->initNode()->configuration());
    sol->appendPath((*problem->steeringMethod())(*q, *q));
  }
  roadmap->resetGoalNodes();  // remove the temporary goal node
  return sol;
}
}  // namespace problemTarget
}  // namespace core
}  // namespace hpp
