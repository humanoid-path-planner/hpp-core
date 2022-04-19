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

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/plan-and-optimize.hh>

namespace hpp {
namespace core {

void PlanAndOptimize::oneStep() { pathPlanner_->oneStep(); }

void PlanAndOptimize::startSolve() { pathPlanner_->startSolve(); }

PathVectorPtr_t PlanAndOptimize::finishSolve(const PathVectorPtr_t& path) {
  PathVectorPtr_t result = path;
  for (Optimizers_t::iterator itOpt = optimizers_.begin();
       itOpt != optimizers_.end(); ++itOpt) {
    result = (*itOpt)->optimize(result);
  }
  return result;
}

void PlanAndOptimize::addPathOptimizer(const PathOptimizerPtr_t& optimizer) {
  optimizers_.push_back(optimizer);
}

PlanAndOptimizePtr_t PlanAndOptimize::create(
    const PathPlannerPtr_t& pathPlanner) {
  PlanAndOptimize* ptr = new PlanAndOptimize(pathPlanner);
  return PlanAndOptimizePtr_t(ptr);
}

PlanAndOptimize::PlanAndOptimize(const PathPlannerPtr_t& pathPlanner)
    : PathPlanner(pathPlanner->problem(), pathPlanner->roadmap()),
      pathPlanner_(pathPlanner),
      optimizers_() {}

}  // namespace core
}  // namespace hpp
