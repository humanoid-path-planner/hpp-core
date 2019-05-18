//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana
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

#ifndef HPP_CORE_VISIBILITY_PRM_PLANNER_HH
# define HPP_CORE_VISIBILITY_PRM_PLANNER_HH

# include <boost/tuple/tuple.hpp>
# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Generic implementation of visibility-PRM algorithm,
    /// based on guard nodes (which cannot see each other) and 
    /// connection nodes between guards.
    class HPP_CORE_DLLAPI VisibilityPrmPlanner : public PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static VisibilityPrmPlannerPtr_t createWithRoadmap
	(const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Return shared pointer to new object.
      static VisibilityPrmPlannerPtr_t create (const Problem& problem);
      /// One step of extension.
      virtual void oneStep ();
    protected:
      /// Constructor
      VisibilityPrmPlanner (const Problem& problem, 
			    const RoadmapPtr_t& roadmap);
      /// Constructor with roadmap
      VisibilityPrmPlanner (const Problem& problem);
      /// Store weak pointer to itself
      void init (const VisibilityPrmPlannerWkPtr_t& weak);
    private:
      typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      VisibilityPrmPlannerWkPtr_t weakPtr_;
      DelayedEdges_t delayedEdges_;
      std::map <NodePtr_t, bool> nodeStatus_; // true for guard node

      /// Return true if the configuration is visible from the given 
      /// connected component.
      bool visibleFromCC (const Configuration_t q, 
			  const ConnectedComponentPtr_t cc);
      
      /// Apply the problem constraints on a given configuration qTo by 
      /// projecting it on the tangent space of qFrom.
      void applyConstraints (
          const Configuration_t& qFrom, 
          const Configuration_t& qTo,
          Configuration_t& qOut);

      bool constrApply_; // True if applyConstraints has successed
    };
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_VISIBILITY_PRM_PLANNER_HH
