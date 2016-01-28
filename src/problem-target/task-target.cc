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

#include <hpp/core/problem-target/task-target.hh>

#include <stdexcept>

#include <hpp/util/debug.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace problemTarget {
      HPP_DEFINE_REASON_FAILURE (REASON_PROJECTION_FAILED, "Projection failed");
      HPP_DEFINE_REASON_FAILURE (REASON_COLLISION, "Collision");

      TaskTargetPtr_t TaskTarget::create (const PathPlannerPtr_t& planner)
      {
        TaskTarget* tt = new TaskTarget (planner);
        TaskTargetPtr_t shPtr (tt);
        tt->init (shPtr);
        return shPtr;
      }

      void TaskTarget::check () const
      {
        if (!constraints_) {
          std::string msg ("No constraints: task not specified.");
          hppDout (error, msg);
          throw std::runtime_error (msg);
        }
      }

      void TaskTarget::initRoadmap ()
      {
        planner_->roadmap()->resetGoalNodes ();
        std::size_t trials = 1;
        ConfigurationPtr_t ng = generateNewConfig (trials);
        if (ng) {
          planner_->roadmap()->addGoalNode (ng);
          goals_.push_back (ng);
        }
      }

      void TaskTarget::oneStep ()
      {
        if (goals_.size () > planner_->roadmap()->nodes().size() / 10)
          return;
        std::size_t trials = 1;
        ConfigurationPtr_t ng = generateNewConfig (trials);
        if (ng) {
          planner_->roadmap()->addGoalNode (ng);
          goals_.push_back (ng);
        }
      }

      ConfigurationPtr_t TaskTarget::generateNewConfig (std::size_t& tries)
      {
        ConfigurationPtr_t q;
        const ConfigValidationsPtr_t& confValidations =
          planner_->problem().configValidations();
        ValidationReportPtr_t report;
        while (tries > 0) {
          tries--;
          q = planner_->problem().configurationShooter()->shoot ();
          if (constraints_->apply (*q)) {
            if (confValidations->validate (*q, report)) {
              statistics_.addSuccess ();
              return q;
            } else
              statistics_.addFailure (REASON_COLLISION);
          } else statistics_.addFailure (REASON_PROJECTION_FAILED);
        }
        return ConfigurationPtr_t();
      }

      void TaskTarget::addGoalConfig (const ConfigurationPtr_t& config)
      {
        const ConfigValidationsPtr_t& confValidations =
          planner_->problem().configValidations();
        ValidationReportPtr_t report;
        if (constraints_->apply (*config)) {
          if (confValidations->validate (*config, report)) {
            statistics_.addSuccess ();
          } else
            statistics_.addFailure (REASON_COLLISION);
        } else statistics_.addFailure (REASON_PROJECTION_FAILED);
      }
    } // namespace problemTarget
  } // namespace core
} // namespace hpp
