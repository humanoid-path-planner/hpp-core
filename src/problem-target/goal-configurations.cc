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
#include <hpp/core/config-validations.hh>

#include "../astar.hh"

namespace hpp {
  namespace core {
    namespace problemTarget {
      GoalConfigurationsPtr_t GoalConfigurations::create
        (const ProblemPtr_t& problem);
      {
        GoalConfigurations* gc = new GoalConfigurations (problem);
        GoalConfigurationsPtr_t shPtr (gc);
        gc->init (shPtr);
        return shPtr;
      }

      void GoalConfigurations::check () const
      {
        if (goalCfgs_.empty ()) {
          std::string msg ("No goal configurations.");
          hppDout (error, msg);
          throw std::runtime_error (msg);
        }

        ValidationReportPtr_t report;
        const ConfigValidationsPtr_t& confValidations =
          problem_.lock()->configValidations();
        for (Configurations_t::const_iterator it = goalCfgs_.begin ();
            it != goalCfgs_.end (); it++) {
          const ConfigurationPtr_t& goalConf (*it);
          if (!confValidations->validate (*goalConf, report)) {
            std::ostringstream oss;
            oss << *report;
            throw std::runtime_error (oss.str ());
          }
        }
      }

      void GoalConfigurations::initRoadmap (const RoadmapPtr_t& roadmap)
      {
        roadmap_ = roadmap;
        roadmap_->resetGoalNodes ();
        for (Configurations_t::const_iterator itGoal = goalCfgs_.begin ();
            itGoal != goalCfgs_.end (); ++itGoal) {
          // TODO goalNodes_.push_back(roadmap_->addNode (*itGoal));
          goalNodes_.push_back(roadmap_->addGoalNode (*itGoal));
        }
      }

      bool GoalConfigurations::reached () const
      {
        const ConnectedComponentPtr_t ccInit = roadmap_->initNode ()->connectedComponent ();
        for (NodeVector_t::const_iterator itGoal = goalNodes_.begin ();
            itGoal != goalNodes_.end (); ++itGoal) {
          if (ccInit->canReach ((*itGoal)->connectedComponent ())) {
            return true;
          }
        }
        return false;
      }

      PathPtr_t GoalConfigurations::computePath() const
      {
        Astar astar (roadmap(), problem_.distance ());
        return astar.solution ();
      }
    } // namespace problemTarget
  } // namespace core
} // namespace hpp
