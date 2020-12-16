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

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    using pinocchio::displayConfig;

    BiRRTPlannerPtr_t BiRRTPlanner::createWithRoadmap
    (const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap)
    {
      BiRRTPlanner* ptr = new BiRRTPlanner (problem, roadmap);
      return BiRRTPlannerPtr_t (ptr);
    }

    BiRRTPlannerPtr_t BiRRTPlanner::create (const ProblemConstPtr_t& problem)
    {
      BiRRTPlanner* ptr = new BiRRTPlanner (problem);
      return BiRRTPlannerPtr_t (ptr);
    }

    BiRRTPlanner::BiRRTPlanner (const ProblemConstPtr_t& problem):
      PathPlanner (problem),
      configurationShooter_ (problem->configurationShooter()),
      qProj_ (problem->robot ()->configSize ())
    {
    }

    BiRRTPlanner::BiRRTPlanner (const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap):
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem->configurationShooter()),
      qProj_ (problem->robot ()->configSize ())
    {
    }

    void BiRRTPlanner::init (const BiRRTPlannerWkPtr_t& weak)
    {
      PathPlanner::init (weak);
      weakPtr_ = weak;
    }

    PathPtr_t BiRRTPlanner::extendInternal (const SteeringMethodPtr_t& sm, Configuration_t& qProj_, const NodePtr_t& near,
                    const Configuration_t& target, bool reverse)
    {
        const ConstraintSetPtr_t& constraints (sm->constraints ());
        if (constraints)
        {
            ConfigProjectorPtr_t configProjector (constraints->configProjector ());
            if (configProjector)
            {
                configProjector->projectOnKernel (*(near->configuration ()), target,
                        qProj_);
            }
            else
            {
                qProj_ = target;
            }
            if (constraints->apply (qProj_))
            {
                return reverse ? (*sm) (qProj_, *(near->configuration ())) : (*sm) (*(near->configuration ()), qProj_);
            }
            else
            {
                return PathPtr_t ();
            }
        }
        return reverse ? (*sm) (target, *(near->configuration ())) : (*sm) (*(near->configuration ()), target);
    }


    /// One step of extension.
    void BiRRTPlanner::startSolve()
    {
        PathPlanner::startSolve();
        startComponent_ = roadmap()->initNode()->connectedComponent();
        for(NodeVector_t::const_iterator cit = roadmap()->goalNodes().begin();
            cit != roadmap()->goalNodes().end(); ++cit)
        {
            endComponents_.push_back((*cit)->connectedComponent());
        }
    }

    void BiRRTPlanner::oneStep ()
    {
        PathPtr_t validPath, path;
        PathValidationPtr_t pathValidation (problem()->pathValidation ());
        value_type distance;
        NodePtr_t near, reachedNodeFromStart;
        bool startComponentConnected(false), pathValidFromStart(false);
        ConfigurationPtr_t q_new;
        // first try to connect to start component
        Configuration_t q_rand;
        configurationShooter_->shoot (q_rand);
        near = roadmap()->nearestNode (q_rand, startComponent_, distance);
        path = extendInternal (problem()->steeringMethod(), qProj_, near,
			       q_rand);
        if (path)
        {
            PathValidationReportPtr_t report;
            pathValidFromStart = pathValidation->validate (path, false, validPath, report);
            if(validPath){
              // Insert new path to q_near in roadmap
              value_type t_final = validPath->timeRange ().second;
              if (t_final != path->timeRange ().first)
              {
                  startComponentConnected = true;
                  q_new = ConfigurationPtr_t (new Configuration_t(validPath->end ()));
                  reachedNodeFromStart = roadmap()->addNodeAndEdge(near, q_new, validPath);
              }
            }
        }

        // now try to connect to end components
        for (std::vector<ConnectedComponentPtr_t>::const_iterator itcc =
           endComponents_.begin ();
         itcc != endComponents_.end (); ++itcc)
        {
            near = roadmap()->nearestNode (q_rand, *itcc, distance,true);
            path = extendInternal (problem()->steeringMethod(), qProj_, near,
				   q_rand, true);
            if (path)
            {
                PathValidationReportPtr_t report;
                if(pathValidation->validate (path, true, validPath, report) && pathValidFromStart)
                {
                    // we won, a path is found
                    roadmap()->addEdge(reachedNodeFromStart, near, validPath);
                    return;
                }
                else if (validPath)
                {
                    value_type t_final = validPath->timeRange ().second;
                    if (t_final != path->timeRange ().first)
                    {
                        ConfigurationPtr_t q_newEnd = ConfigurationPtr_t (new Configuration_t(validPath->initial()));
                        NodePtr_t newNode = roadmap()->addNodeAndEdge (q_newEnd,near,validPath);
                        // now try to connect both nodes
                        if(startComponentConnected)
                        {
                            path = (*(problem()->steeringMethod()))
			      (*q_new, *q_newEnd);
                            if(path && pathValidation->validate (path, false, validPath, report))
                            {
                                roadmap()->addEdge (reachedNodeFromStart, newNode, path);
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
  } // namespace core
} // namespace hpp
