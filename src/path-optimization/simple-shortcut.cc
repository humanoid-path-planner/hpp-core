//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#include <hpp/core/path-optimization/simple-shortcut.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
SimpleShortcutPtr_t SimpleShortcut::create(const ProblemConstPtr_t& problem) {
  SimpleShortcut* ptr = new SimpleShortcut(problem);
  return SimpleShortcutPtr_t(ptr);
}

SimpleShortcut::SimpleShortcut(const ProblemConstPtr_t& problem)
    : PathOptimizer(problem) {}

PathVectorPtr_t SimpleShortcut::optimize(const PathVectorPtr_t& path) {
  RoadmapPtr_t roadmap(
      Roadmap::create(problem()->distance(), problem()->robot()));
  std::vector<NodePtr_t> nodes;
  Configuration_t qPtr(path->initial());
  roadmap->initNode(qPtr);
  NodePtr_t node(roadmap->initNode());
  nodes.push_back(node);
  for (std::size_t i = 0; i < path->numberPaths(); ++i) {
    PathPtr_t p(path->pathAtRank(i));
    qPtr = Configuration_t(p->end());
    node = roadmap->addNodeAndEdge(node, qPtr, p);
    nodes.push_back(node);
  }
  roadmap->addGoalNode(node->configuration());
  PathValidationPtr_t pv(problem()->pathValidation());
  for (std::size_t i = 0; i < nodes.size() - 1; ++i) {
    for (std::size_t j = i + 2; j < nodes.size(); ++j) {
      PathPtr_t path(
          steer(nodes[i]->configuration(), nodes[j]->configuration()));
      PathValidationReportPtr_t report;
      PathPtr_t unused;
      if ((path) && (pv->validate(path, false, unused, report))) {
        roadmap->addEdge(nodes[i], nodes[j], path);
      }
    }
  }
  PathVectorPtr_t result(problem()->target()->computePath(roadmap));
  assert(result);
  return result;
}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
