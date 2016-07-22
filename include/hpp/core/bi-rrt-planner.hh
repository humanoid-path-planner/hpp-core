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

#ifndef HPP_CORE_BIRRT_PLANNER_HH
# define HPP_CORE_BIRRT_PLANNER_HH

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Implementation of directional bi-RRT algorithm
    /// maintaining only two connected components for
    /// respectively the start and goal configurations
    class HPP_CORE_DLLAPI BiRRTPlanner : public PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static BiRRTPlannerPtr_t createWithRoadmap
	(const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Return shared pointer to new object.
      static BiRRTPlannerPtr_t create (const Problem& problem);
      /// One step of extension.
      virtual void startSolve();
      /// One step of extension.
      virtual void oneStep ();
    protected:
      /// Constructor
      BiRRTPlanner (const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Constructor with roadmap
      BiRRTPlanner (const Problem& problem);
      /// Store weak pointer to itself
      void init (const BiRRTPlannerWkPtr_t& weak);
      PathPtr_t extendInternal (const SteeringMethodPtr_t& sm, Configuration_t& qProj_, const NodePtr_t& near,
                      const ConfigurationPtr_t& target, bool reverse=false);

      ConfigurationShooterPtr_t configurationShooter_;
      ConnectedComponentPtr_t startComponent_;
      std::vector<ConnectedComponentPtr_t> endComponents_;
    private:
      mutable Configuration_t qProj_;
      BiRRTPlannerWkPtr_t weakPtr_;
    };
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_BIRRT_PLANNER_HH
