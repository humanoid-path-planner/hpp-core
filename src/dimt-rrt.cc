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

#include <boost/tuple/tuple.hpp>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/dimt-rrt.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/kinodynamic-distance.hh>

namespace hpp {
namespace core {
using model::displayConfig;

DimtRRTPtr_t DimtRRT::createWithRoadmap
(const Problem& problem, const RoadmapPtr_t& roadmap)
{
    DimtRRT* ptr = new DimtRRT (problem, roadmap);
    return DimtRRTPtr_t (ptr);
}

DimtRRTPtr_t DimtRRT::create (const Problem& problem)
{
    DimtRRT* ptr = new DimtRRT (problem);
    return DimtRRTPtr_t (ptr);
}

DimtRRT::DimtRRT (const Problem& problem):
    BiRRTPlanner (problem),
    qProj_ (problem.robot ()->configSize ())
{
}

DimtRRT::DimtRRT (const Problem& problem,
                  const RoadmapPtr_t& roadmap) :
    BiRRTPlanner (problem, roadmap),
    qProj_ (problem.robot ()->configSize ())
{
}

void DimtRRT::init (const DimtRRTWkPtr_t& weak)
{
    BiRRTPlanner::init (weak);
    weakPtr_ = weak;
}


void DimtRRT::oneStep ()
{
    hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ new Step ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    PathPtr_t validPath, path;
    PathValidationPtr_t pathValidation (problem ().pathValidation ());
    value_type distance;
    NodePtr_t near, reachedNodeFromStart;
    bool startComponentConnected(false), pathValidFromStart(false),pathValidFromEnd(false);
    ConfigurationPtr_t q_new;
    // first try to connect to start component
    ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
    hppDout(info,"Random configuration : "<<displayConfig(*q_rand));
    near = roadmap()->nearestNode (q_rand, startComponent_, distance);
    path = extendInternal (problem().steeringMethod(), qProj_, near, q_rand);
    if (path)
    {
        PathValidationReportPtr_t report;
        pathValidFromStart = pathValidation->validate (path, false, validPath, report);
        // Insert new path to q_near in roadmap
        if(validPath){
            value_type t_final = validPath->timeRange ().second;
            if (t_final != path->timeRange ().first)
            {
                startComponentConnected = true;
                q_new = ConfigurationPtr_t (new Configuration_t(validPath->end ()));
                reachedNodeFromStart = roadmap()->addNodeAndEdge(near, q_new, validPath);
                hppDout(info,"~~~~~~~~~~~~~~~~~~~~ New node added to start component : "<<displayConfig(*q_new));

            }
        }
    }

    hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Try to connect end component ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

    // now try to connect to end components
    for (std::vector<ConnectedComponentPtr_t>::const_iterator itcc =
       endComponents_.begin ();
     itcc != endComponents_.end (); ++itcc)
    {
        near = roadmap()->nearestNode (q_rand, *itcc, distance,true);
        path = extendInternal (problem().steeringMethod(), qProj_, near, q_rand, true);
        if (path)
        {
            PathValidationReportPtr_t report;
            pathValidFromEnd = pathValidation->validate (path, true, validPath, report);
            if(pathValidFromEnd && pathValidFromStart)
            {
                // we won, a path is found
                roadmap()->addEdge(reachedNodeFromStart, near, validPath);
                hppDout(info,"~~~~~~~~~~~~~~~~~~~~ Start and goal component connected !!!!!! "<<displayConfig(*q_new));
                return;
            }
            else if (validPath)
            {
                value_type t_final = validPath->timeRange ().second;
                if (t_final != path->timeRange ().first)
                {
                    ConfigurationPtr_t q_newEnd = ConfigurationPtr_t (new Configuration_t(validPath->initial()));
                    NodePtr_t newNode = roadmap()->addNodeAndEdge(q_newEnd, near, validPath);
                    hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~ New node added to end component : "<<displayConfig(*q_newEnd));

                    // now try to connect both nodes
                    if(startComponentConnected)
                    {
                        path = (*(problem().steeringMethod())) (*q_new, *q_newEnd);
                        if(path && pathValidation->validate (path, false, validPath, report))
                        {
                            roadmap()->addEdge (reachedNodeFromStart, newNode, path);
                            hppDout(info,"~~~~~~~~ both new nodes connected together !!!!!! "<<displayConfig(*q_new));
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
