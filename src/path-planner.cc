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

#include <hpp/util/debug.hh>

#include <hpp/core/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {

    PathPlanner::PathPlanner (const Problem& problem) :
      problem_ (problem), roadmap_ (Roadmap::create (problem.distance (),
						     problem.robot())),
      interrupt_ (false),
      maxIterations_ (std::numeric_limits <unsigned long int>::max ())
    {
    }

    PathPlanner::PathPlanner (const Problem& problem,
			      const RoadmapPtr_t& roadmap) :
      problem_ (problem), roadmap_ (roadmap),
      interrupt_ (false),
      maxIterations_ (std::numeric_limits <unsigned long int>::max ())
    {
    }

    void PathPlanner::init (const PathPlannerWkPtr_t& weak)
    {
      weakPtr_ = weak;
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
      roadmap()->resetGoalNodes ();
      roadmap()->initNode (problem_.initConfig ());
      const Configurations_t goals (problem_.goalConfigs ());
      for (Configurations_t::const_iterator itGoal = goals.begin ();
          itGoal != goals.end (); ++itGoal) {
        roadmap()->addGoalNode (*itGoal);
      }

      problem_.target()->check(roadmap());
    }

    PathVectorPtr_t PathPlanner::solve ()
    {
      interrupt_ = false;
      bool solved = false;
      unsigned long int nIter (0);
      startSolve ();
      tryDirectPath ();
      solved = problem_.target()->reached (roadmap());
      if (solved ) {
	hppDout (info, "tryDirectPath succeeded");
      }
      if (interrupt_) throw std::runtime_error ("Interruption");
      while (!solved) {
	std::ostringstream oss;
    if (maxIterations_ !=  std::numeric_limits <unsigned long int>::infinity () && nIter >= maxIterations_) {
	  oss << "Maximal number of iterations reached: " << maxIterations_;
	  throw std::runtime_error (oss.str ().c_str ());
	}
	oneStep ();
	++nIter;
        solved = problem_.target()->reached (roadmap());
	if (interrupt_) throw std::runtime_error ("Interruption");
      }
      PathVectorPtr_t planned =  computePath ();
      return finishSolve (planned);
    }

    void PathPlanner::interrupt ()
    {
      interrupt_ = true;
    }

    void PathPlanner::maxIterations (const unsigned long int& n)
    {
      maxIterations_ = n;
    }

    PathVectorPtr_t PathPlanner::computePath () const
    {
      return problem_.target()->computePath(roadmap());
    }

    PathVectorPtr_t PathPlanner::finishSolve (const PathVectorPtr_t& path)
    {
      return path;
    }

    void PathPlanner::tryDirectPath ()
    {
      // call steering method here to build a direct conexion
      const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathProjectorPtr_t pathProjector (problem ().pathProjector ());
      PathPtr_t validPath, projPath, path;
      NodePtr_t initNode = roadmap ()->initNode();
      for (NodeVector_t::const_iterator itn = roadmap ()->goalNodes ().begin();
	   itn != roadmap ()->goalNodes ().end (); ++itn) {
	ConfigurationPtr_t q1 ((initNode)->configuration ());
	ConfigurationPtr_t q2 ((*itn)->configuration ());
	path = (*sm) (*q1, *q2);
        if (!path) continue;
        if (pathProjector) {
          if (!pathProjector->apply (path, projPath)) continue;
        } else {
          projPath = path;
        }
        if (projPath) {
	  PathValidationReportPtr_t report;
          bool pathValid = pathValidation->validate (projPath, false, validPath,
						     report);
          if (pathValid && validPath->length() > 0) {
            roadmap ()->addEdge (initNode, *itn, projPath);
            roadmap ()->addEdge (*itn, initNode, projPath->reverse());
          }
        }
      }
    }
  } //   namespace core
} // namespace hpp
