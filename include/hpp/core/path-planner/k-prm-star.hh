//
// Copyright (c) 2018 CNRS
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

#ifndef HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH
# define HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    namespace pathPlanner {
      /// k-PRM* path planning algorithm
      /// as described in https://arxiv.org/pdf/1105.1186.pdf.
      class HPP_CORE_DLLAPI kPrmStar : public PathPlanner
      {
      public:
        /// Constant kPRM = 2 e
        static const double kPRM;
        typedef PathPlanner Parent_t;
        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        static kPrmStarPtr_t create (const Problem& problem);
        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        static kPrmStarPtr_t createWithRoadmap (const Problem& problem,
                                                const RoadmapPtr_t& roadmap);
      /// Initialize the problem resolution
      ///  \li call parent implementation
      ///  \li get number nodes in problem parameter map
      virtual void startSolve ();
      /// One step of the algorithm
      virtual void oneStep ();

      protected:
        /// Protected constructor
        /// \param problem the path planning problem
        kPrmStar (const Problem& problem);
        /// Protected constructor
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        kPrmStar (const Problem& problem, const RoadmapPtr_t& roadmap);
        /// Store weak pointer to itself
        void init (const kPrmStarWkPtr_t& weak);

      private:
        enum STATE {
          BUILD_ROADMAP,
          CONNECT_INIT_GOAL,
          FAILURE
        }; // enum STATE
        STATE state_;
        /// Generate random free configurations 10 by 10
        void generateRandomConfig ();
        /// Link each node with closest neighbors
        void linkNodes ();
        /// Connect initial and goal configurations to roadmap
        void connectInitAndGoal ();
        /// Connect node to k closest neighbors in the roadmap
        void connectNodeToClosestNeighbors (const NodePtr_t& node);
        /// Configuration shooter to uniformly shoot random configurations
        ConfigurationShooterPtr_t shooter_;
        /// Number of nodes to create
        std::size_t numberNodes_;
        /// Weak pointer to itself
        kPrmStarWkPtr_t weak_;
      }; // class kPrmStar
    } // namespace pathPlanner
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_PLANNER_K_PRM_STAR_HH
