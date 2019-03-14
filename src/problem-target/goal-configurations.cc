// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/problem-target/goal-configurations.hh>

#include <stdexcept>

#include <hpp/util/debug.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/config-validations.hh>

#include "../astar.hh"

namespace hpp {
  namespace core {
    namespace problemTarget {
      GoalConfigurationsPtr_t GoalConfigurations::create
        (const ProblemPtr_t& problem)
      {
        GoalConfigurations* gc = new GoalConfigurations (problem);
        GoalConfigurationsPtr_t shPtr (gc);
        gc->init (shPtr);
        return shPtr;
      }

      void GoalConfigurations::check (const RoadmapPtr_t& /*roadmap*/) const
      {
      }

      bool GoalConfigurations::reached (const RoadmapPtr_t& roadmap) const
      {
        const ConnectedComponentPtr_t ccInit = roadmap->initNode ()->connectedComponent ();
        const NodeVector_t& goals = roadmap->goalNodes();
        for (NodeVector_t::const_iterator _goal = goals.begin (); _goal != goals.end (); ++_goal) {
          if (ccInit->canReach ((*_goal)->connectedComponent ())) {
            return true;
          }
        }
        return false;
      }

      PathVectorPtr_t GoalConfigurations::computePath(const RoadmapPtr_t& roadmap) const
      {
        ProblemPtr_t problem (problem_.lock());
        assert (problem);
        Astar astar (roadmap, problem->distance ());
        PathVectorPtr_t sol = PathVector::create (
            problem->robot()->configSize(), problem->robot()->numberDof());
        astar.solution (sol);
        // This happens when q_init == q_goal
        if (sol->numberPaths() == 0) {
          ConfigurationPtr_t q (roadmap->initNode()->configuration ());
          sol->appendPath(
              (*problem->steeringMethod()) (*q, *q)
              );
        }
        return sol;
      }
    } // namespace problemTarget
  } // namespace core
} // namespace hpp
