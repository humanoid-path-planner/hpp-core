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

#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
using pinocchio::displayConfig;

BiRRTPlannerPtr_t BiRRTPlanner::createWithRoadmap(
    const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap) {
  BiRRTPlanner* ptr = new BiRRTPlanner(problem, roadmap);
  return BiRRTPlannerPtr_t(ptr);
}

BiRRTPlannerPtr_t BiRRTPlanner::create(const ProblemConstPtr_t& problem) {
  BiRRTPlanner* ptr = new BiRRTPlanner(problem);
  return BiRRTPlannerPtr_t(ptr);
}

BiRRTPlanner::BiRRTPlanner(const ProblemConstPtr_t& problem)
    : PathPlanner(problem),
      configurationShooter_(problem->configurationShooter()),
      qProj_(problem->robot()->configSize()) {}

BiRRTPlanner::BiRRTPlanner(const ProblemConstPtr_t& problem,
                           const RoadmapPtr_t& roadmap)
    : PathPlanner(problem, roadmap),
      configurationShooter_(problem->configurationShooter()),
      qProj_(problem->robot()->configSize()) {}

void BiRRTPlanner::init(const BiRRTPlannerWkPtr_t& weak) {
  PathPlanner::init(weak);
  weakPtr_ = weak;
}

PathPtr_t BiRRTPlanner::extendInternal(const SteeringMethodPtr_t& sm,
                                       Configuration_t& qProj_,
                                       const NodePtr_t& near,
                                       const Configuration_t& target,
                                       bool reverse) {
  const ConstraintSetPtr_t& constraints(sm->constraints());
  if (constraints) {
    ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(near->configuration(), target,
                                       qProj_);
    } else {
      qProj_ = target;
    }
    if (constraints->apply(qProj_)) {
      return reverse ? (*sm)(qProj_, near->configuration())
                     : (*sm)(near->configuration(), qProj_);
    } else {
      return PathPtr_t();
    }
  }
  return reverse ? (*sm)(target, near->configuration())
                 : (*sm)(near->configuration(), target);
}

/// One step of extension.
void BiRRTPlanner::startSolve() {
  PathPlanner::startSolve();
  startComponent_ = roadmap()->initNode()->connectedComponent();
  for (NodeVector_t::const_iterator cit = roadmap()->goalNodes().begin();
       cit != roadmap()->goalNodes().end(); ++cit) {
    endComponents_.push_back((*cit)->connectedComponent());
  }
}

void BiRRTPlanner::oneStep() {
  PathPtr_t validPath, path;
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  value_type distance;
  NodePtr_t near, reachedNodeFromStart;
  bool startComponentConnected(false), pathValidFromStart(false);
  Configuration_t q_new;
  // first try to connect to start component
  Configuration_t q_rand;
  configurationShooter_->shoot(q_rand);
  near = roadmap()->nearestNode(q_rand, startComponent_, distance);
  path = extendInternal(problem()->steeringMethod(), qProj_, near, q_rand);
  if (path) {
    PathValidationReportPtr_t report;
    pathValidFromStart =
        pathValidation->validate(path, false, validPath, report);
    if (validPath) {
      // Insert new path to q_near in roadmap
      value_type t_final = validPath->timeRange().second;
      if (t_final != path->timeRange().first) {
        startComponentConnected = true;
        q_new = validPath->end();
        reachedNodeFromStart =
            roadmap()->addNodeAndEdge(near, q_new, validPath);
      }
    }
  }

  // now try to connect to end components
  for (std::vector<ConnectedComponentPtr_t>::const_iterator itcc =
           endComponents_.begin();
       itcc != endComponents_.end(); ++itcc) {
    near = roadmap()->nearestNode(q_rand, *itcc, distance, true);
    path =
        extendInternal(problem()->steeringMethod(), qProj_, near, q_rand, true);
    if (path) {
      PathValidationReportPtr_t report;
      if (pathValidation->validate(path, true, validPath, report) &&
          pathValidFromStart) {
        // we won, a path is found
        roadmap()->addEdge(reachedNodeFromStart, near, validPath);
        return;
      } else if (validPath) {
        value_type t_final = validPath->timeRange().second;
        if (t_final != path->timeRange().first) {
          Configuration_t q_newEnd = validPath->initial();
          NodePtr_t newNode =
              roadmap()->addNodeAndEdge(q_newEnd, near, validPath);
          // now try to connect both nodes
          if (startComponentConnected) {
            path = (*(problem()->steeringMethod()))(q_new, q_newEnd);
            if (path &&
                pathValidation->validate(path, false, validPath, report)) {
              roadmap()->addEdge(reachedNodeFromStart, newNode, path);
              return;
            }
          }
        }
      }
    }
  }
}
}  // namespace core
}  // namespace hpp
