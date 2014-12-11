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

#ifndef HPP_CORE_PLAN_AND_OPTIMIZE_HH
# define HPP_CORE_PLAN_AND_OPTIMIZE_HH

# include <vector>
# include <hpp/core/config.hh>
# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Path planner and optimizer
    ///
    /// Plans a path and iteratively applies a series of optimizer on
    /// the result.
    class HPP_CORE_DLLAPI PlanAndOptimize : public PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static PlanAndOptimizePtr_t create (const PathPlannerPtr_t& pathPlanner);
      /// Call internal path planner implementation
      virtual void startSolve ();
      /// One iteration of path planning or path optimization
      virtual void oneStep ();
      /// Optimize planned path
      virtual PathVectorPtr_t finishSolve (const PathVectorPtr_t& path);
      void addPathOptimizer (const PathOptimizerPtr_t& optimizer);
    protected:
      PlanAndOptimize (const PathPlannerPtr_t& pathPlanner);
    private:
      typedef std::vector <PathOptimizerPtr_t> Optimizers_t;
      const PathPlannerPtr_t pathPlanner_;
      Optimizers_t optimizers_;
    }; // class PlanAndOptimize
    /// \}
  } // namespace core
} // namespace hpp

# endif // HPP_CORE_PLAN_AND_OPTIMIZE_HH
