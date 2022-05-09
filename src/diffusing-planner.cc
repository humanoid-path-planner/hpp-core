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

#include <hpp/core/diffusing-planner.hh>

#include <tuple>
#include <iterator>

#include <pinocchio/math/quaternion.hpp>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace {
      using pinocchio::displayConfig;

      HPP_DEFINE_TIMECOUNTER(oneStep);
      HPP_DEFINE_TIMECOUNTER(extend);
      HPP_DEFINE_TIMECOUNTER(tryConnect);
      HPP_DEFINE_TIMECOUNTER(validatePath);
      HPP_DEFINE_TIMECOUNTER(delayedEdges);
    }

    DiffusingPlannerPtr_t DiffusingPlanner::createWithRoadmap
    (const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap)
    {
      DiffusingPlanner* ptr = new DiffusingPlanner (problem, roadmap);
      return DiffusingPlannerPtr_t (ptr);
    }

    DiffusingPlannerPtr_t DiffusingPlanner::create (const ProblemConstPtr_t& problem)
    {
      DiffusingPlanner* ptr = new DiffusingPlanner (problem);
      return DiffusingPlannerPtr_t (ptr);
    }

    DiffusingPlanner::DiffusingPlanner (const ProblemConstPtr_t& problem):
      PathPlanner (problem),
      configurationShooter_ (problem->configurationShooter()),
      qProj_ (problem->robot ()->configSize ())
    {
    }

    DiffusingPlanner::DiffusingPlanner (const ProblemConstPtr_t& problem,
					const RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem->configurationShooter()),
      qProj_ (problem->robot ()->configSize ())
    {
    }

    void DiffusingPlanner::init (const DiffusingPlannerWkPtr_t& weak)
    {
      PathPlanner::init (weak);
      weakPtr_ = weak;
    }

    bool belongs (const ConfigurationPtr_t& q, const Nodes_t& nodes)
    {
      for (Nodes_t::const_iterator itNode = nodes.begin ();
	   itNode != nodes.end (); ++itNode) {
	if (*((*itNode)->configuration ()) == *q) return true;
      }
      return false;
    }

    PathPtr_t DiffusingPlanner::extend (const NodePtr_t& near,
					const Configuration_t& target)
    {
      const SteeringMethodPtr_t& sm (problem()->steeringMethod ());
      const ConstraintSetPtr_t& constraints (sm->constraints ());
      if (constraints) {
	ConfigProjectorPtr_t configProjector (constraints->configProjector ());
	if (configProjector) {
    assert (isNormalized(problem()->robot(), target, PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE));
    assert (isNormalized(problem()->robot(), *(near->configuration()), PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE));
	  configProjector->projectOnKernel (*(near->configuration ()), target,
					    qProj_);
    assert (isNormalized(problem()->robot(), qProj_, PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE));
	} else {
	  qProj_ = target;
	}
	if (!constraints->apply (qProj_)) {
	  return PathPtr_t ();
	}
      } else {
        qProj_ = target;
      }
      assert(!qProj_.hasNaN());
      // Here, qProj_ is a configuration that satisfies the constraints
      // or target if there are no constraints.
      PathPtr_t path = (*sm) (*(near->configuration ()), qProj_);
      if (!path) {
        return PathPtr_t ();
      }
      value_type stepLength = problem()->getParameter
	("DiffusingPlanner/extensionStepLength").floatValue();
      if (stepLength > 0 && path->length() > stepLength) {
        value_type t0 = path->timeRange().first;
        path = path->extract(t0, t0 + stepLength);
      }
      PathProjectorPtr_t pp = problem()->pathProjector();
      if (pp) {
        PathPtr_t proj;
        pp->apply (path, proj);
        return proj;
      }
      return path;
    }

    void DiffusingPlanner::startSolve ()
    {
      Parent_t::startSolve ();
      problemTarget::GoalConfigurationsPtr_t gc
        (HPP_DYNAMIC_PTR_CAST(problemTarget::GoalConfigurations,
                              problem()->target()));
      if (!gc) {
        throw std::logic_error("DiffusingPlanner only accepts goals defined "
                               "by goal configurations.");
      }
    }

    /// This method performs one step of RRT extension as follows
    ///  1. a random configuration "q_rand" is shot,
    ///  2. for each connected component,
    ///    2.1. the closest node "q_near" is chosen,
    ///    2.2. "q_rand" is projected first on the tangent space of the
    ///         non-linear constraint at "q_near", this projection yields
    ///         "q_tmp", then "q_tmp" is projected on the non-linear constraint
    ///         manifold as "q_proj" (method extend)
    ///    2.3. the steering method is called between "q_near" and "q_proj" that
    ///         returns "path",
    ///    2.4. a valid connected part of "path", called "validPath" starting at
    ///         "q_near" is extracted, if "path" is valid (collision free),
    ///         the full "path" is returned, "q_new" is the end configuration of
    ///         "validPath",
    ///    2.5  a new node containing "q_new" is added to the connected
    ///         component and a new edge is added between nodes containing
    ///         "q_near" and "q_new".
    ///  3. Try to connect new nodes together using the steering method and
    ///     the current PathValidation instance.
    ///
    ///  Note that edges are actually added to the roadmap after step 2 in order
    ///  to avoid iterating on the list of connected components while modifying
    ///  this list.

    void DiffusingPlanner::oneStep ()
    {
      HPP_START_TIMECOUNTER(oneStep);

      value_type stepRatio = problem()->getParameter
	("DiffusingPlanner/extensionStepRatio").floatValue();

      typedef std::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;
      DevicePtr_t robot (problem()->robot ());
      PathValidationPtr_t pathValidation (problem()->pathValidation ());
      Nodes_t newNodes, nearestNeighbors;
      PathPtr_t validPath, path;
      // Pick a random node
      Configuration_t q_rand;
      configurationShooter_->shoot (q_rand);
      //
      // First extend each connected component toward q_rand
      //
      for (ConnectedComponents_t::const_iterator itcc =
	     roadmap ()->connectedComponents ().begin ();
	   itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
	// Find nearest node in roadmap
	value_type distance;
	NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
        nearestNeighbors.push_back (near);
        HPP_START_TIMECOUNTER(extend);
	path = extend (near, q_rand);
        HPP_STOP_TIMECOUNTER(extend);
	if (path) {
	  PathValidationReportPtr_t report;
          HPP_START_TIMECOUNTER(validatePath);
	  bool pathValid = pathValidation->validate (path, false, validPath,
						     report);
          HPP_STOP_TIMECOUNTER(validatePath);
	  // Insert new path to q_near in roadmap
	  value_type t_final = validPath->timeRange ().second;
	  if (t_final != path->timeRange ().first) {
            if (!pathValid && stepRatio > 0 && stepRatio < 1.) {
              value_type t0 = validPath->timeRange().first;
              validPath = validPath->extract(t0, t0 + validPath->length()*stepRatio);
            }
	    ConfigurationPtr_t q_new (new Configuration_t
				      (validPath->end ()));
	    if (!pathValid || !belongs (q_new, newNodes)) {
	      newNodes.push_back (roadmap ()->addNodeAndEdges
				  (near, q_new, validPath));
	    } else {
	      // Store edges to add for later insertion.
	      // Adding edges while looping on connected components is indeed
	      // not recommended.
	      delayedEdges.push_back (DelayedEdge_t (near, q_new, validPath));
	    }
	  }
	}
      }
      // Insert delayed edges
      HPP_START_TIMECOUNTER(delayedEdges);
      for (const auto& edge : delayedEdges) {
	const NodePtr_t& near = std::get<0>(edge);
	const ConfigurationPtr_t& q_new = std::get<1>(edge);
	const PathPtr_t& validPath = std::get<2>(edge);
	NodePtr_t newNode = roadmap ()->addNode (q_new);
	roadmap ()->addEdge (near, newNode, validPath);
	roadmap ()->addEdge (newNode, near, validPath->reverse());
      }
      HPP_STOP_TIMECOUNTER(delayedEdges);

      //
      // Second, try to connect new nodes together
      //
      HPP_START_TIMECOUNTER(tryConnect);
      const SteeringMethodPtr_t& sm (problem()->steeringMethod ());
      for (Nodes_t::const_iterator itn1 = newNodes.begin ();
	   itn1 != newNodes.end (); ++itn1) {
        /// Try connecting to the other new nodes.
	for (Nodes_t::const_iterator itn2 = std::next (itn1);
	     itn2 != newNodes.end (); ++itn2) {
	  ConfigurationPtr_t q1 ((*itn1)->configuration ());
	  ConfigurationPtr_t q2 ((*itn2)->configuration ());
	  assert (*q1 != *q2);
	  path = (*sm) (*q1, *q2);
          if (!path) continue;

          PathProjectorPtr_t pp = problem()->pathProjector();
          if (pp) {
            PathPtr_t proj;
            // If projection failed, continue
            if (!pp->apply (path, proj)) continue;
            path = proj;
          }

	  PathValidationReportPtr_t report;
          HPP_START_TIMECOUNTER(validatePath);
	  bool valid = pathValidation->validate (path, false, validPath, report);
          HPP_STOP_TIMECOUNTER(validatePath);
          if (valid) {
	    roadmap ()->addEdge (*itn1, *itn2, path);
	    roadmap ()->addEdge (*itn2, *itn1, path->reverse ());
          } else if (validPath && validPath->length () > 0) {
            // A -> B
            ConfigurationPtr_t cfg (new Configuration_t (validPath->end()));
            roadmap ()->addNodeAndEdges (*itn1, cfg, validPath);
          }
	}
        /// Try connecting this node to the list of nearest neighbors.
        const ConnectedComponentPtr_t& cc1 = (*itn1)->connectedComponent();
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
	for (Nodes_t::const_iterator itn2 = nearestNeighbors.begin();
	     itn2 != nearestNeighbors.end (); ++itn2) {
          if (cc1 == (*itn2)->connectedComponent ()) continue;
	  ConfigurationPtr_t q2 ((*itn2)->configuration ());
	  assert (*q1 != *q2);
	  path = (*sm) (*q1, *q2);
          if (!path) continue;

          PathProjectorPtr_t pp = problem()->pathProjector();
          if (pp) {
            PathPtr_t proj;
            // If projection failed, continue
            if (!pp->apply (path, proj)) continue;
            path = proj;
          }

	  PathValidationReportPtr_t report;
          HPP_START_TIMECOUNTER(validatePath);
	  bool valid = pathValidation->validate (path, false, validPath, report);
          HPP_STOP_TIMECOUNTER(validatePath);
          if (valid) {
	    roadmap ()->addEdge (*itn1, *itn2, path);
	    roadmap ()->addEdge (*itn2, *itn1, path->reverse ());
          } else if (validPath && validPath->length () > 0) {
            // A -> B
            ConfigurationPtr_t cfg (new Configuration_t (validPath->end()));
            roadmap ()->addNodeAndEdges (*itn1, cfg, validPath);
          }
	}
      }
      HPP_STOP_TIMECOUNTER(tryConnect);

      HPP_STOP_TIMECOUNTER(oneStep);

      HPP_DISPLAY_TIMECOUNTER(oneStep);
      HPP_DISPLAY_TIMECOUNTER(extend);
      HPP_DISPLAY_TIMECOUNTER(validatePath);
      HPP_DISPLAY_TIMECOUNTER(delayedEdges);
      HPP_DISPLAY_TIMECOUNTER(tryConnect);
    }

    void DiffusingPlanner::configurationShooter
    (const ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }

    HPP_START_PARAMETER_DECLARATION(DiffusingPlanner)
    Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
          "DiffusingPlanner/extensionStepLength",
          "Extension step length. "
          "Not used if negative.",
          Parameter(-1.)));
    Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
          "DiffusingPlanner/extensionStepRatio",
          "When path from q_near to q_rand is in collision, keep only this "
          "amount of the valid part. "
          "Should be in ]0,1[. Not used if negative.",
          Parameter(-1.)));
    HPP_END_PARAMETER_DECLARATION(DiffusingPlanner)
  } // namespace core
} // namespace hpp
