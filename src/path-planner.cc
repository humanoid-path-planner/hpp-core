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

#include <hpp/core/path-planner.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/steering-method.hh>
#include "astar.hh"

namespace hpp {
  namespace core {

    PathPlanner::PathPlanner (const Problem& problem) :
      problem_ (problem), roadmap_ (Roadmap::create (problem.distance (), 
						     problem.robot())),
      interrupt_ (false)
    {
    }
    
    PathPlanner::PathPlanner (const Problem& problem,
			      const RoadmapPtr_t& roadmap) :
      problem_ (problem), roadmap_ (roadmap),
      interrupt_ (false)
    {
    }
    
    const RoadmapPtr_t& PathPlanner::roadmap () const
    {
      return roadmap_;
    }

    const Problem& PathPlanner::problem () const
    {
      return problem_;
    }

    void PathPlanner::startSolve ()
    {
      problem_.checkProblem ();
      // Tag init and goal configurations in the roadmap
      roadmap_->resetGoalNodes ();
      roadmap_->initNode (problem_.initConfig ());
      const Configurations_t goals (problem_.goalConfigs ());
      for (Configurations_t::const_iterator itGoal = goals.begin ();
	   itGoal != goals.end (); itGoal++) {
	roadmap_->addGoalNode (*itGoal);
      }
    }

    PathVectorPtr_t PathPlanner::solve ()
    {
      interrupt_ = false;
      bool solved = false;
      startSolve ();
      tryDirectPath();
      solved = pathExists ();
      if (interrupt_) throw std::runtime_error ("Interruption");
      while (!solved) {
	oneStep ();
	solved = pathExists ();
	if (interrupt_) throw std::runtime_error ("Interruption");
      }
      PathVectorPtr_t planned =  computePath ();
      return finishSolve (planned);
    }

    void PathPlanner::interrupt ()
    {
      interrupt_ = true;
    }

    bool PathPlanner::pathExists () const
    {
      for (Nodes_t::const_iterator itGoal = roadmap_->goalNodes ().begin ();
	   itGoal != roadmap_->goalNodes ().end (); itGoal++) {
	if ((*itGoal)->connectedComponent () ==
	    roadmap_->initNode ()->connectedComponent ()) {
	  return true;
	}
      }
      return false;
    }

    PathVectorPtr_t PathPlanner::computePath () const
    {
      Astar astar (roadmap_, problem_.distance ());
      return astar.solution ();
    }

    void PathPlanner::tryDirectPath ()
    {
      // call steering method here to build a direct conexion 
      const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathPtr_t validPath, path;
      NodePtr_t initNode = roadmap ()->initNode();
      for (Nodes_t::const_iterator itn = roadmap ()->goalNodes ().begin();
	   itn != roadmap ()->goalNodes ().end (); ++itn) {
	ConfigurationPtr_t q1 ((initNode)->configuration ());
	ConfigurationPtr_t q2 ((*itn)->configuration ());
	assert (*q1 != *q2);
	path = (*sm) (*q1, *q2);
	bool pathValid = pathValidation->validate (path, false, validPath);
	if (pathValid && validPath->timeRange ().second != 
	    path->timeRange ().first) {
	  roadmap ()->addEdge (initNode, *itn, path);
	  interval_t timeRange = path->timeRange ();
	  roadmap ()->addEdge (*itn, initNode, path->extract
			       (interval_t (timeRange.second,
					    timeRange.first)));
	}
      }
    }
    
  } //   namespace core
} // namespace hpp
