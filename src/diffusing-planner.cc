//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/core/diffusing-planner.hh>

#include <boost/tuple/tuple.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>

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
    (const Problem& problem, const RoadmapPtr_t& roadmap)
    {
      DiffusingPlanner* ptr = new DiffusingPlanner (problem, roadmap);
      return DiffusingPlannerPtr_t (ptr);
    }

    DiffusingPlannerPtr_t DiffusingPlanner::create (const Problem& problem)
    {
      DiffusingPlanner* ptr = new DiffusingPlanner (problem);
      return DiffusingPlannerPtr_t (ptr);
    }

    DiffusingPlanner::DiffusingPlanner (const Problem& problem):
      PathPlanner (problem),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ())
    {
    }

    DiffusingPlanner::DiffusingPlanner (const Problem& problem,
					const RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ())
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
					const ConfigurationPtr_t& target)
    {
      const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      const ConstraintSetPtr_t& constraints (sm->constraints ());
      if (constraints) {
	ConfigProjectorPtr_t configProjector (constraints->configProjector ());
	if (configProjector) {
	  configProjector->projectOnKernel (*(near->configuration ()), *target,
					    qProj_);
	} else {
	  qProj_ = *target;
	}
	if (constraints->apply (qProj_)) {
	  return (*sm) (*(near->configuration ()), qProj_);
	} else {
	  return PathPtr_t ();
	}
      }
      return (*sm) (*(near->configuration ()), *target);
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

      typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;
      DevicePtr_t robot (problem ().robot ());
      PathValidationPtr_t pathValidation (problem ().pathValidation ());
      Nodes_t newNodes, nearestNeighbors;
      PathPtr_t validPath, path;
      // Pick a random node
      ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
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
      for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
	   itEdge != delayedEdges.end (); ++itEdge) {
	const NodePtr_t& near = itEdge-> get <0> ();
	const ConfigurationPtr_t& q_new = itEdge-> get <1> ();
	const PathPtr_t& validPath = itEdge-> get <2> ();
	NodePtr_t newNode = roadmap ()->addNode (q_new);
	roadmap ()->addEdge (near, newNode, validPath);
	roadmap ()->addEdge (newNode, near, validPath->reverse());
      }
      HPP_STOP_TIMECOUNTER(delayedEdges);

      //
      // Second, try to connect new nodes together
      //
      HPP_START_TIMECOUNTER(tryConnect);
      const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      for (Nodes_t::const_iterator itn1 = newNodes.begin ();
	   itn1 != newNodes.end (); ++itn1) {
        /// Try connecting to the other new nodes.
	for (Nodes_t::const_iterator itn2 = boost::next (itn1);
	     itn2 != newNodes.end (); ++itn2) {
	  ConfigurationPtr_t q1 ((*itn1)->configuration ());
	  ConfigurationPtr_t q2 ((*itn2)->configuration ());
	  assert (*q1 != *q2);
	  path = (*sm) (*q1, *q2);
	  PathValidationReportPtr_t report;
          if (!path) continue;
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


  } // namespace core
} // namespace hpp
