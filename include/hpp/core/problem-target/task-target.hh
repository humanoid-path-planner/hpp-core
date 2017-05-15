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

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/problem-target.hh>

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
          static TaskTargetPtr_t create (const ProblemPtr_t& problem);

          /// Check if the problem target is well specified.
          void check (const RoadmapPtr_t& roadmap) const;

          /// Check whether the problem is solved.
          bool reached (const RoadmapPtr_t& roadmap) const;

          PathVectorPtr_t computePath(const RoadmapPtr_t& roadmap) const;

          void constraints (const ConstraintSetPtr_t& c)
          {
            constraints_ = c;
          }

        protected:
          /// Constructor
          TaskTarget (const ProblemPtr_t& problem)
            : ProblemTarget (problem)
          {}

        private:
          ConstraintSetPtr_t constraints_;
      }; // class TaskTarget
      /// \}
    } // namespace problemTarget
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_TARGET_TASK_TARGET_HH
