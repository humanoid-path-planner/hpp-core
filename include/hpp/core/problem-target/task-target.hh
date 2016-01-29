//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_PROBLEM_TARGET_TASK_TARGET_HH
# define HPP_CORE_PROBLEM_TARGET_TASK_TARGET_HH

# include <hpp/core/problem-target.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

# include <hpp/statistics/success-bin.hh>

namespace hpp {
  namespace core {
    namespace problemTarget {
      /// \addtogroup path_planning
      /// \{

      /// Task target
      ///
      /// This class defines a goal using constraints. The set of goal
      /// configurations is a submanifold of the full configuration space.
      /// \warning So far, this feature is not taken into account by
      /// most planners. The supported planners are:
      /// - DiffusingPlanner
      class HPP_CORE_DLLAPI TaskTarget : public ProblemTarget {
        public:
          static TaskTargetPtr_t create (const PathPlannerPtr_t& planner);

          /// Check if the problem target is well specified.
          void check () const;

          /// Try sampling one configuration and add it as goal.
          void initRoadmap ();

          /// Try sampling one configuration and add it as goal.
          void oneStep ();

          void constraints (const ConstraintSetPtr_t& c)
          {
            constraints_ = c;
            indexInInitcc_ = 0;
          }

          /// Apply the constraints to the configuration and
          /// add it to the goals.
          /// \warning the input configuration is modified.
          void addGoalConfig (const ConfigurationPtr_t& config);

          void resetGoalConfig ()
          {
            goals_.clear ();
          }

        protected:
          /// Constructor
          TaskTarget (const PathPlannerPtr_t& planner)
            : ProblemTarget (planner)
            , indexInInitcc_ (0)
            , statistics_ ("Task target")
          {}

        private:
          ConfigurationPtr_t generateNewConfig (std::size_t& tries);

          ConfigurationPtr_t shootConfig ();

          bool impl_addGoalConfig (const ConfigurationPtr_t& config);

          ConstraintSetPtr_t constraints_;

          std::size_t indexInInitcc_;

          ::hpp::statistics::SuccessStatistics statistics_;
      }; // class TaskTarget
      /// \}
    } // namespace problemTarget
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_TARGET_TASK_TARGET_HH
