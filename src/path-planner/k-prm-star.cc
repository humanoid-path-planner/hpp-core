//
// Copyright (c) 2018 CNRS
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

#include <cmath>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-planning-failed.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

namespace hpp {
namespace core {
namespace pathPlanner {

const double kPrmStar::kPRM = 2 * exp(1);

kPrmStarPtr_t kPrmStar::create(const ProblemConstPtr_t& problem) {
  kPrmStarPtr_t shPtr(new kPrmStar(problem));
  shPtr->init(shPtr);
  return shPtr;
}

kPrmStarPtr_t kPrmStar::createWithRoadmap(const ProblemConstPtr_t& problem,
                                          const RoadmapPtr_t& roadmap) {
  kPrmStarPtr_t shPtr(new kPrmStar(problem, roadmap));
  shPtr->init(shPtr);
  return shPtr;
}

void kPrmStar::startSolve() {
  Parent_t::startSolve();
  numberNodes_ = problem()->getParameter("kPRM*/numberOfNodes").intValue();
  if (numberNodes_ == 0) {
    std::ostringstream oss;
    oss << "kPrmStar: Number nodes should be positive, got " << numberNodes_;
    throw std::runtime_error(oss.str().c_str());
  }
  numberNeighbors_ =
      (size_type)floor((kPRM * log((value_type)numberNodes_)) + .5);
  if (roadmap()->nodes().size() >= numberNodes_) {
    state_ = CONNECT_INIT_GOAL;
  } else {
    state_ = BUILD_ROADMAP;
  }
}

void kPrmStar::tryConnectInitAndGoals() {}

void kPrmStar::oneStep() {
  std::ostringstream oss;
  // Shoot valid random configurations
  switch (state_) {
    case BUILD_ROADMAP:
      generateRandomConfig();
      break;
    case LINK_NODES:
      linkNodes();
      break;
    case CONNECT_INIT_GOAL:
      connectInitAndGoal();
      state_ = FAILURE;
      break;
    case FAILURE:
      oss << "kPRM* failed to solve problem with " << numberNodes_ << " nodes.";
      throw path_planning_failed(oss.str().c_str());
  }
}

void kPrmStar::generateRandomConfig() {
  // shoot a valid random configuration
  Configuration_t qrand;
  // Report of configuration validation: unused here
  ValidationReportPtr_t validationReport;
  // Configuration validation methods associated to the problem
  ConfigValidationsPtr_t configValidations(problem()->configValidations());
  // Get the constraints the robot is subject to
  ConstraintSetPtr_t constraints(problem()->constraints());
  // Get the problem shooter
  ConfigurationShooterPtr_t shooter = problem()->configurationShooter();
  // Get roadmap
  RoadmapPtr_t r(roadmap());
  if (r->nodes().size() < numberNodes_) {
    size_type nbTry = 0;
    bool valid(false);
    // After 10000 trials throw if no valid configuration has been found.
    do {
      shooter->shoot(qrand);
      valid = (!constraints || constraints->apply(qrand));
      if (valid) valid = configValidations->validate(qrand, validationReport);
      nbTry++;
    } while (!valid && nbTry < 10000);
    if (!valid) {
      throw path_planning_failed(
          "Failed to generate free configuration after 10000 trials.");
    }
    r->addNode(qrand);
  } else {
    state_ = LINK_NODES;
    linkingNodeIt_ = r->nodes().begin();
    neighbors_ = roadmap()->nearestNodes((*linkingNodeIt_)->configuration(),
                                         numberNeighbors_);
    itNeighbor_ = neighbors_.begin();
  }
}

void kPrmStar::linkNodes() {
  // Get roadmap
  RoadmapPtr_t r(roadmap());
  if (linkingNodeIt_ != r->nodes().end()) {
    if (connectNodeToClosestNeighbors(*linkingNodeIt_)) {
      ++linkingNodeIt_;
      if (linkingNodeIt_ != r->nodes().end()) {
        neighbors_ = roadmap()->nearestNodes((*linkingNodeIt_)->configuration(),
                                             numberNeighbors_);
        // Connect current node with closest neighbors
        itNeighbor_ = neighbors_.begin();
      }
    } else {
      ++itNeighbor_;
    }
  } else {
    state_ = CONNECT_INIT_GOAL;
  }
}

bool kPrmStar::connectNodeToClosestNeighbors(const NodePtr_t& node) {
  // Retrieve the path validation algorithm associated to the problem
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  // Retrieve the steering method
  SteeringMethodPtr_t sm(problem()->steeringMethod());
  // Retrieve the constraints the robot is subject to
  ConstraintSetPtr_t constraints(problem()->constraints());
  // Retrieve path projector
  PathProjectorPtr_t pathProjector(problem()->pathProjector());

  if (itNeighbor_ != neighbors_.end()) {
    // Connect only nodes that are not already connected
    if (!(*itNeighbor_)->isOutNeighbor(node) && (node != *itNeighbor_)) {
      PathPtr_t p(
          (*sm)(node->configuration(), (*itNeighbor_)->configuration()));
      PathValidationReportPtr_t report;
      PathPtr_t validPart, projected;
      if (p) {
        bool success;
        if (pathProjector) {
          success = pathProjector->apply(p, projected);
        } else {
          projected = p;
          success = true;
        }
        if (success) {
          if (pathValidation->validate(projected, false, validPart, report)) {
            roadmap()->addEdges(node, *itNeighbor_, projected);
          }
        }
      }
    }
    return false;
  } else {
    // itNeighbor_ reached the end
    return true;
  }
}

void kPrmStar::connectInitAndGoal() {
  NodePtr_t initNode(roadmap()->initNode());
  if (initNode->outEdges().empty()) {
    neighbors_ =
        roadmap()->nearestNodes(initNode->configuration(), numberNeighbors_);
    // Connect current node with closest neighbors
    for (itNeighbor_ = neighbors_.begin(); itNeighbor_ != neighbors_.end();
         ++itNeighbor_) {
      connectNodeToClosestNeighbors(initNode);
    }
  }
  for (NodeVector_t::const_iterator itn(roadmap()->goalNodes().begin());
       itn != roadmap()->goalNodes().end(); ++itn) {
    neighbors_ =
        roadmap()->nearestNodes((*itn)->configuration(), numberNeighbors_);
    if ((*itn)->inEdges().empty()) {
      for (itNeighbor_ = neighbors_.begin(); itNeighbor_ != neighbors_.end();
           ++itNeighbor_) {
        connectNodeToClosestNeighbors(*itn);
      }
    }
  }
}

kPrmStar::STATE kPrmStar::getComputationState() const { return state_; }

kPrmStar::kPrmStar(const ProblemConstPtr_t& problem)
    : Parent_t(problem), state_(BUILD_ROADMAP) {}

kPrmStar::kPrmStar(const ProblemConstPtr_t& problem,
                   const RoadmapPtr_t& roadmap)
    : Parent_t(problem, roadmap), state_(BUILD_ROADMAP) {}

void kPrmStar::init(const kPrmStarWkPtr_t& weak) {
  Parent_t::init(weak);
  weak_ = weak;
}

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(kPrmStar)
Problem::declareParameter(ParameterDescription(
    Parameter::INT, "kPRM*/numberOfNodes",
    "The desired number of nodes in the roadmap.", Parameter((size_type)100)));
HPP_END_PARAMETER_DECLARATION(kPrmStar)
}  // namespace pathPlanner
}  // namespace core
}  // namespace hpp
