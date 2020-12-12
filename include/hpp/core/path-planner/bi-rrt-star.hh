//
// Copyright (c) 2020 CNRS
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

#ifndef HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH
# define HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    namespace pathPlanner {
      HPP_PREDEF_CLASS (BiRrtStar);
      typedef boost::shared_ptr <BiRrtStar> BiRrtStarPtr_t;

      /// Bi-RRT* path planning algorithm
      /// as described in http://www.golems.org/papers/AkgunIROS11-sampling.pdf
      class HPP_CORE_DLLAPI BiRrtStar : public PathPlanner
      {
      public:
        typedef PathPlanner Parent_t;

        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        static BiRrtStarPtr_t create (const ProblemConstPtr_t& problem);
        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        static BiRrtStarPtr_t createWithRoadmap
	  (const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);

        /// Initialize the problem resolution
        ///  \li call parent implementation
        void startSolve ();
        /// One step of the algorithm
        void oneStep ();

      protected:
        /// Protected constructor
        /// \param problem the path planning problem
        BiRrtStar (const ProblemConstPtr_t& problem);
        /// Protected constructor
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        BiRrtStar (const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap);
        /// Store weak pointer to itself
        void init (const BiRrtStarWkPtr_t& weak);

      private:
        typedef std::map<NodePtr_t, EdgePtr_t> ParentMap_t;

        Configuration_t sample ();

        /// Get cost to reach \c n
        value_type cost(NodePtr_t n);

        /// Set cost to reach \c n
        void cost(NodePtr_t n, value_type c);

        /// internal function to build path.
        /// It returns an empty path on failure.
        /// If \c validatePath is true, it returns only the valid part.
        /// If \c maxLength is positive, the returned path will be at most of this length.
        PathPtr_t buildPath(const Configuration_t& q0, const Configuration_t& q1,
            value_type maxLength, bool validatePath);

        bool extend (NodePtr_t target, ParentMap_t& parentMap, Configuration_t& q);

        bool connect (NodePtr_t cc, ParentMap_t& parentMap, const Configuration_t& q);

        bool improve (const Configuration_t& q);

        value_type gamma_;
        /// Maximal path length with using function \ref extend.
        value_type extendMaxLength_;

        NodePtr_t roots_[2];

        /// store relation <child, parent> that brings to node \c roots_[i]
        std::vector<ParentMap_t> toRoot_;

        /// Weak pointer to itself
        BiRrtStarWkPtr_t weak_;
      }; // class BiRrtStar
    } // namespace pathPlanner
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_PLANNER_BI_RRT_STAR_HH
