//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana
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

#include <stdio.h>
#include <time.h>

#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/visibility-prm-planner.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
using pinocchio::displayConfig;

VisibilityPrmPlannerPtr_t VisibilityPrmPlanner::createWithRoadmap(
    const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap) {
  VisibilityPrmPlanner* ptr = new VisibilityPrmPlanner(problem, roadmap);
  return VisibilityPrmPlannerPtr_t(ptr);
}

VisibilityPrmPlannerPtr_t VisibilityPrmPlanner::create(
    const ProblemConstPtr_t& problem) {
  VisibilityPrmPlanner* ptr = new VisibilityPrmPlanner(problem);
  return VisibilityPrmPlannerPtr_t(ptr);
}

VisibilityPrmPlanner::VisibilityPrmPlanner(const ProblemConstPtr_t& problem)
    : PathPlanner(problem) {}

VisibilityPrmPlanner::VisibilityPrmPlanner(const ProblemConstPtr_t& problem,
                                           const RoadmapPtr_t& roadmap)
    : PathPlanner(problem, roadmap) {}

void VisibilityPrmPlanner::init(const VisibilityPrmPlannerWkPtr_t& weak) {
  PathPlanner::init(weak);
  weakPtr_ = weak;
}

bool VisibilityPrmPlanner::visibleFromCC(const Configuration_t q,
                                         const ConnectedComponentPtr_t cc) {
  PathPtr_t validPart;
  bool found = false;
  value_type length = std::numeric_limits<value_type>::infinity();
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  SteeringMethodPtr_t sm(problem()->steeringMethod());
  RoadmapPtr_t r(roadmap());
  DelayedEdge_t delayedEdge;

  for (NodeVector_t::const_iterator n_it = cc->nodes().begin();
       n_it != cc->nodes().end(); ++n_it) {
    if (nodeStatus_[*n_it]) {  // only iterate on guard nodes
      ConfigurationPtr_t qCC = (*n_it)->configuration();
      PathPtr_t path = (*sm)(q, *qCC);
      PathValidationReportPtr_t report;
      if (path && pathValidation->validate(path, false, validPart, report)) {
        // q and qCC see each other
        if (path->length() < length) {
          length = path->length();
          // Save shortest edge
          delayedEdge = DelayedEdge_t(*n_it, make_shared<Configuration_t>(q),
                                      path->reverse());
        }
        found = true;
      }
    }
  }
  if (found) {
    // Store shortest delayed edge in list
    delayedEdges_.push_back(delayedEdge);
    return true;
  } else
    return false;
}

void VisibilityPrmPlanner::applyConstraints(const Configuration_t& qFrom,
                                            const Configuration_t& qTo,
                                            Configuration_t& qout) {
  ConstraintSetPtr_t constraints(problem()->constraints());
  if (constraints) {
    ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      constrApply_ = false;  // while apply has not successed
      configProjector->projectOnKernel(qFrom, qTo, qout);
      if (constraints->apply(qout)) {
        constrApply_ = true;
        return;
      }
    }
  }
  qout = qTo;
}

void VisibilityPrmPlanner::oneStep() {
  DevicePtr_t robot(problem()->robot());
  ConfigurationShooterPtr_t configurationShooter(
      problem()->configurationShooter());
  ConfigValidationsPtr_t configValidations(problem()->configValidations());
  RoadmapPtr_t r(roadmap());
  value_type count;     // number of times q has been seen
  constrApply_ = true;  // stay true if no constraint in Problem
  Configuration_t q_init(*(r->initNode()->configuration())),
      q_proj(robot->configSize()), q_rand(robot->configSize());

  /* Initialization of guard status */
  nodeStatus_[r->initNode()] = true;  // init node is guard
  for (NodeVector_t::const_iterator itg = r->goalNodes().begin();
       itg != r->goalNodes().end(); ++itg) {
    nodeStatus_[*itg] = true;  // goal nodes are guards
  }

  // Shoot random config as long as not collision-free
  ValidationReportPtr_t report;
  do {
    configurationShooter->shoot(q_rand);
    applyConstraints(q_init, q_rand, q_proj);
    robot->currentConfiguration(q_proj);
    robot->computeForwardKinematics();
  } while (!configValidations->validate(q_proj, report) || !constrApply_);
  count = 0;

  for (ConnectedComponents_t::const_iterator itcc =
           r->connectedComponents().begin();
       itcc != r->connectedComponents().end(); ++itcc) {
    ConnectedComponentPtr_t cc = *itcc;
    if (visibleFromCC(q_proj, cc)) {
      // delayedEdges_ will completed if visible
      count++;  // count how many times q has been seen
    }
  }

  if (count == 0) {                          // q not visible from anywhere
    NodePtr_t newNode = r->addNode(q_proj);  // add q as a guard node
    nodeStatus_[newNode] = true;
    hppDout(info, "q is a guard node: " << displayConfig(q_proj));
  }
  if (count > 1) {  // q visible several times
    // Insert delayed edges from list and add q as a connection node
    for (const auto& edge : delayedEdges_) {
      const NodePtr_t& near = std::get<0>(edge);
      const ConfigurationPtr_t& q_new = std::get<1>(edge);
      const PathPtr_t& validPath = std::get<2>(edge);
      NodePtr_t newNode = r->addNode(q_new);
      nodeStatus_[newNode] = false;
      r->addEdge(near, newNode, validPath);
      r->addEdge(newNode, near, validPath->reverse());
      hppDout(info, "connection between q1: "
                        << displayConfig(*(near->configuration()))
                        << "and q2: " << displayConfig(*q_new));
    }
  }
  delayedEdges_.clear();
}

}  // namespace core
}  // namespace hpp
