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

#ifndef HPP_CORE_PROBLEM_TARGET_HH
# define HPP_CORE_PROBLEM_TARGET_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Problem target
    ///
    /// This abstract class defines the goal to be reached by a planning
    /// algorithm.
    class HPP_CORE_DLLAPI ProblemTarget {
    public:
      virtual ~ProblemTarget () {};

      /// Check if the problem target is well specified.
      virtual void check (const RoadmapPtr_t& roadmap) const = 0;

      /// Check whether the problem is solved.
      virtual bool reached (const RoadmapPtr_t& roadmap) const = 0;

      /// Returns the solution path found.
      /// Should be called when reached() returns true.
      virtual PathVectorPtr_t computePath(const RoadmapPtr_t& roadmap) const = 0;

      /// Set the problem
      void problem (const ProblemPtr_t& problem)
      {
        problem_ = problem;
      }

    protected:
      /// Constructor
      ProblemTarget (const ProblemPtr_t& problem)
        : problem_ (problem)
      {}

      /// Store weak pointer to itself
      void init (const ProblemTargetWkPtr_t& weak)
      {
        weakPtr_ = weak;
      }

      /// Reference to the planner for access to problem and roadmap
      ProblemWkPtr_t problem_;

      /// Store weak pointer to itself
      ProblemTargetWkPtr_t weakPtr_;

    private:
    }; // class ProblemTarget
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_TARGET_HH
