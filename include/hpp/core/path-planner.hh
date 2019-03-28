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

#ifndef HPP_CORE_PATH_PLANNER_HH
# define HPP_CORE_PATH_PLANNER_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Path planner
    ///
    /// Algorithm that computes a path between an initial configuration and a
    /// set of goal configurations.
    class HPP_CORE_DLLAPI PathPlanner {
    public:
      virtual ~PathPlanner () {};

      /// Get roadmap
      virtual const RoadmapPtr_t& roadmap () const;
      /// Get problem
      const Problem& problem () const;
      /// Initialize the problem resolution
      ///  \li Set initial and and goal nodes,
      ///  \li check problem consistency
      virtual void startSolve ();
      /// Solve
      ///
      /// Call methods
      /// \li startSolve,
      /// \li oneStep until a solution is found,
      /// \li finishSolve.
      /// Users can implement themselves the loop to avoid being trapped
      /// in an infinite loop when no solution is found.
      virtual PathVectorPtr_t solve ();
      /// Try to make direct connection between init and goal
      /// configurations, in order to avoid a random shoot.
      virtual void tryDirectPath() HPP_CORE_DEPRECATED;
      /// Try to connect initial and goal configurations to existing roadmap
      virtual void tryConnectInitAndGoals ();

      /// User implementation of one step of resolution
      virtual void oneStep () = 0;
      /// Post processing of the resulting path
      virtual PathVectorPtr_t finishSolve (const PathVectorPtr_t& path);
      /// Interrupt path planning
      void interrupt ();
      /// Set maximal number of iterations
      void maxIterations (const unsigned long int& n);
      /// set time out (in seconds)
      void timeOut(const double& timeOut);
      /// Find a path in the roadmap and transform it in trajectory
      PathVectorPtr_t computePath () const;
    protected:
      /// Constructor
      ///
      /// Create a new roadmap
      PathPlanner (const Problem& problem);
      /// Constructor
      ///
      /// Store a given roadmap.
      PathPlanner (const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Store weak pointer to itself
      void init (const PathPlannerWkPtr_t& weak);
    private:
      /// Reference to the problem
      const Problem& problem_;
      /// Pointer to the roadmap.
      const RoadmapPtr_t roadmap_;
      bool interrupt_;
      /// Maximal number of iterations to solve a problem
      /// reaching this bound raises an exception.
      unsigned long int maxIterations_;
      /// Time out (in seconds) before interrupting the planning
      double timeOut_;

      /// Store weak pointer to itself
      PathPlannerWkPtr_t weakPtr_;
    }; // class PathPlanner
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_PLANNER_HH
