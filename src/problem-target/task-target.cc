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

#include <hpp/core/node.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace problemTarget {
      namespace {
        HPP_DEFINE_REASON_FAILURE (REASON_PROJECTION_FAILED, "Projection failed");
        HPP_DEFINE_REASON_FAILURE (REASON_COLLISION, "Collision");
      }

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
        std::size_t trials = 2;
        generateNewConfig (trials);
      }

      void TaskTarget::oneStep ()
      {
        if (goals_.size () > planner_->roadmap()->nodes().size() / 10)
          return;
        std::size_t trials = 1;
        generateNewConfig (trials);
      }

      ConfigurationPtr_t TaskTarget::generateNewConfig (std::size_t& tries)
      {
        ConfigurationPtr_t q;
        while (tries > 0) {
          tries--;
          q = shootConfig ();
          if (impl_addGoalConfig (q)) return q;
        }
        return ConfigurationPtr_t();
      }

      ConfigurationPtr_t TaskTarget::shootConfig ()
      {
        ConfigurationPtr_t q;
        const RoadmapPtr_t& r = planner_->roadmap ();
        const NodeVector_t& initCCnodes =
          r->initNode()->connectedComponent()->nodes();
        if (initCCnodes.size() < indexInInitcc_) {
          // The list of nodes must have changed.
          // Try again from initial configuration.
          indexInInitcc_ = 0;
          hppDout (info, "Connected component of init node has been shrinked. "
             "Reinititializing index.");
        }
        if (initCCnodes.size() > indexInInitcc_) {
          q = ConfigurationPtr_t (new Configuration_t (
                *(initCCnodes[indexInInitcc_]->configuration())
              ));
          indexInInitcc_++;
        } else {
          q = planner_->problem().configurationShooter()->shoot ();
        }
        return q;
      }

      void TaskTarget::addGoalConfig (const ConfigurationPtr_t& config)
      {
        impl_addGoalConfig (config);
      }

      inline bool TaskTarget::impl_addGoalConfig (const ConfigurationPtr_t& config)
      {
        const ConfigValidationsPtr_t& confValidations =
          planner_->problem().configValidations();
        ValidationReportPtr_t report;
        if (constraints_->apply (*config)) {
          if (confValidations->validate (*config, report)) {
            statistics_.addSuccess ();
            planner_->roadmap()->addGoalNode (config);
            goals_.push_back (config);
            return true;
          } else
            statistics_.addFailure (REASON_COLLISION);
        } else statistics_.addFailure (REASON_PROJECTION_FAILED);
        return false;
      }
    } // namespace problemTarget
  } // namespace core
} // namespace hpp
