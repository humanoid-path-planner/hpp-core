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

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/plan-and-optimize.hh>

namespace hpp {
  namespace core {

    void PlanAndOptimize::oneStep ()
    {
      pathPlanner_->oneStep ();
    }

    void PlanAndOptimize::startSolve ()
    {
      pathPlanner_->startSolve ();
    }

    PathVectorPtr_t PlanAndOptimize::finishSolve (const PathVectorPtr_t& path)
    {
      PathVectorPtr_t result = path;
      for (Optimizers_t::iterator itOpt = optimizers_.begin ();
	   itOpt != optimizers_.end (); itOpt++) {
	result = (*itOpt)->optimize (result);
      }
      return result;
    }

    void PlanAndOptimize::addPathOptimizer
    (const PathOptimizerPtr_t& optimizer)
    {
      optimizers_.push_back (optimizer);
    }

    PlanAndOptimizePtr_t
    PlanAndOptimize::create (const PathPlannerPtr_t& pathPlanner)
    {
      PlanAndOptimize* ptr = new PlanAndOptimize (pathPlanner);
      return PlanAndOptimizePtr_t (ptr);
    }

    PlanAndOptimize::PlanAndOptimize (const PathPlannerPtr_t& pathPlanner) :
      PathPlanner (pathPlanner->problem (), pathPlanner->roadmap ()),
      pathPlanner_ (pathPlanner), optimizers_ ()
    {
    }

  } // namespace core
} // namespace hpp
