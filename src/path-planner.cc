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

#include <hpp/core/edge.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-planning-failed.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <tuple>

#include "astar.hh"

namespace hpp {
namespace core {
unsigned long int uint_infty =
    std::numeric_limits<unsigned long int>::infinity();
value_type float_infty = std::numeric_limits<value_type>::infinity();

PathPlanner::PathPlanner(const ProblemConstPtr_t& problem)
    : problem_(problem),
      roadmap_(Roadmap::create(problem->distance(), problem->robot())),
      interrupt_(false),
      maxIterations_(uint_infty),
      timeOut_(float_infty),
      stopWhenProblemIsSolved_(true) {
  assert(problem_.lock());
}

PathPlanner::PathPlanner(const ProblemConstPtr_t& problem,
                         const RoadmapPtr_t& roadmap)
    : problem_(problem),
      roadmap_(roadmap),
      interrupt_(false),
      maxIterations_(uint_infty),
      timeOut_(float_infty),
      stopWhenProblemIsSolved_(true) {
  assert(problem_.lock());
}

PathPlanner::~PathPlanner() {}

void PathPlanner::init(const PathPlannerWkPtr_t& weak) { weakPtr_ = weak; }

const RoadmapPtr_t& PathPlanner::roadmap() const { return roadmap_; }

ProblemConstPtr_t PathPlanner::problem() const { return problem_.lock(); }

void PathPlanner::startSolve() {
  problem()->checkProblem();
  // Tag init and goal configurations in the roadmap
  roadmap()->resetGoalNodes();
  roadmap()->initNode(problem()->initConfig());
  problemTarget::GoalConfigurationsPtr_t gc(HPP_DYNAMIC_PTR_CAST(
      problemTarget::GoalConfigurations, problem()->target()));
  if (gc) {
    const Configurations_t goals(gc->configurations());
    for (Configurations_t::const_iterator itGoal = goals.begin();
         itGoal != goals.end(); ++itGoal) {
      roadmap()->addGoalNode(*itGoal);
    }
  }
  problem()->target()->check(roadmap());
}

PathVectorPtr_t PathPlanner::solve() {
  namespace bpt = boost::posix_time;

  int status = 0;
  // 0 = execute one more step
  // 1 = pb solved
  // 2 = throw exception

  interrupt_ = false;
  bool solved = false;
  unsigned long int nIter(0);
  bpt::ptime timeStart(bpt::microsec_clock::universal_time());
  startSolve();
  tryConnectInitAndGoals();
  // We choose to stop if a direct path solves the problem.
  // We could also respect the stopWhenLimitReached_ attribute.
  // It is ambiguous what should be done as it is case dependent.
  // If the intent is to build a roadmap, then we should not stop.
  // If the intent is to solve a optimal planning problem, then we should stop.
  solved = problem()->target()->reached(roadmap());
  if (solved) {
    hppDout(info, "tryConnectInitAndGoals succeeded");
  }
  if (interrupt_) throw path_planning_failed("Interruption");
  while (!solved) {
    // Check limits
    std::ostringstream oss;
    if (maxIterations_ != uint_infty && nIter >= maxIterations_)
    // If the maximal nb of iterations is defined and reached
    {
      if (problem()->target()->reached(roadmap()))
        // but a solution has been found
        status = 1;
      else {
        // and no solution has been found
        oss << "Maximal number of iterations reached: " << maxIterations_;
        status = 2;
      }
    }
    bpt::ptime timeStop(bpt::microsec_clock::universal_time());
    value_type elapsed_ms =
        static_cast<value_type>((timeStop - timeStart).total_milliseconds());
    if (elapsed_ms > timeOut_ * 1000)
    // If the time limit has been reached
    {
      if (problem()->target()->reached(roadmap()))
        // but a solution has been found
        status = 1;
      else {
        // and no solution has been found
        oss << "time out (" << timeOut_ << "s) reached after "
            << elapsed_ms * 1e-3 << "s";
        status = 2;
      }
    }

    // Check what to do
    if (status == 1) break;
    if (status == 2) throw path_planning_failed(oss.str().c_str());
    if (status == 0) {  // Execute one step
      hppStartBenchmark(ONE_STEP);
      oneStep();
      hppStopBenchmark(ONE_STEP);
      hppDisplayBenchmark(ONE_STEP);

      // Check if problem is solved.
      ++nIter;
      solved =
          stopWhenProblemIsSolved_ && problem()->target()->reached(roadmap());
    }
    if (interrupt_) throw path_planning_failed("Interruption");
  }
  PathVectorPtr_t planned = computePath();
  return finishSolve(planned);
}

void PathPlanner::interrupt() { interrupt_ = true; }

void PathPlanner::maxIterations(const unsigned long int& n) {
  if (!stopWhenProblemIsSolved_ && n == uint_infty && timeOut_ == float_infty)
    throw std::invalid_argument(
        "stopWhenProblemIsSolved is disabled so "
        "maxIterations or timeOut must be finite.");
  maxIterations_ = n;
}

void PathPlanner::timeOut(const double& time) {
  if (!stopWhenProblemIsSolved_ && maxIterations_ == uint_infty &&
      time == float_infty)
    throw std::invalid_argument(
        "stopWhenProblemIsSolved is disabled so "
        "maxIterations or timeOut must be finite.");
  timeOut_ = time;
}

void PathPlanner::stopWhenProblemIsSolved(bool enable) {
  if (!enable && maxIterations_ == uint_infty && timeOut_ == float_infty)
    throw std::invalid_argument(
        "Disabling stopWhenProblemIsSolved is only "
        "possible when maxIterations or timeOut are set to a finite value.");
  stopWhenProblemIsSolved_ = enable;
}

PathVectorPtr_t PathPlanner::computePath() const {
  return problem()->target()->computePath(roadmap());
}

PathVectorPtr_t PathPlanner::finishSolve(const PathVectorPtr_t& path) {
  return path;
}

void PathPlanner::tryConnectInitAndGoals() {
  // call steering method here to build a direct conexion
  const SteeringMethodPtr_t& sm(problem()->steeringMethod());
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  PathProjectorPtr_t pathProjector(problem()->pathProjector());
  PathPtr_t validPath, projPath, path;
  NodePtr_t initNode = roadmap()->initNode();
  NearestNeighborPtr_t nn(roadmap()->nearestNeighbor());
  // Register edges to add to roadmap and add them after iterating
  // among the connected components.
  typedef std::tuple<NodePtr_t, NodePtr_t, PathPtr_t> FutureEdge_t;
  typedef std::vector<FutureEdge_t> FutureEdges_t;
  FutureEdges_t futureEdges;
  ConnectedComponentPtr_t initCC(initNode->connectedComponent());
  for (ConnectedComponents_t::iterator itCC(
           roadmap()->connectedComponents().begin());
       itCC != roadmap()->connectedComponents().end(); ++itCC) {
    if (*itCC != initCC) {
      value_type d;
      NodePtr_t near(nn->search(initNode->configuration(), *itCC, d, true));
      assert(near);
      Configuration_t q1(initNode->configuration());
      Configuration_t q2(near->configuration());
      path = (*sm)(q1, q2);
      if (!path) continue;
      if (pathProjector) {
        if (!pathProjector->apply(path, projPath)) continue;
      } else {
        projPath = path;
      }
      if (projPath) {
        PathValidationReportPtr_t report;
        bool pathValid =
            pathValidation->validate(projPath, false, validPath, report);
        if (pathValid && validPath->length() > 0) {
          futureEdges.push_back(FutureEdge_t(initNode, near, projPath));
        }
      }
    }
  }
  for (NodeVector_t::const_iterator itn = roadmap()->goalNodes().begin();
       itn != roadmap()->goalNodes().end(); ++itn) {
    ConnectedComponentPtr_t goalCC((*itn)->connectedComponent());
    for (ConnectedComponents_t::iterator itCC(
             roadmap()->connectedComponents().begin());
         itCC != roadmap()->connectedComponents().end(); ++itCC) {
      if (*itCC != goalCC) {
        value_type d;
        NodePtr_t near(nn->search((*itn)->configuration(), *itCC, d, false));
        assert(near);
        Configuration_t q1(near->configuration());
        Configuration_t q2((*itn)->configuration());
        path = (*sm)(q1, q2);
        if (!path) continue;
        if (pathProjector) {
          if (!pathProjector->apply(path, projPath)) continue;
        } else {
          projPath = path;
        }
        if (projPath) {
          PathValidationReportPtr_t report;
          bool pathValid =
              pathValidation->validate(projPath, false, validPath, report);
          if (pathValid && validPath->length() > 0) {
            futureEdges.push_back(FutureEdge_t(near, (*itn), projPath));
          }
        }
      }
    }
  }
  // Add edges
  for (const auto& e : futureEdges)
    roadmap()->addEdge(std::get<0>(e), std::get<1>(e), std::get<2>(e));
}

}  //   namespace core
}  // namespace hpp
