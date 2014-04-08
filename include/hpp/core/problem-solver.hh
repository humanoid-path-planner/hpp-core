//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011 CNRS
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

#ifndef HPP_CORE_PROBLEM_SOLVER_HH
# define HPP_CORE_PROBLEM_SOLVER_HH

# include <hpp/model/fwd.hh>
# include <hpp/core/deprecated.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// Set and solve a path planning problem

    class HPP_CORE_DLLAPI ProblemSolver {
    public:
      /// Constructor
      ProblemSolver ();

      /// Destructor
      virtual ~ProblemSolver ();

      /// Set robot
      void robot (const DevicePtr_t& robot);

      /// Get robot
      const DevicePtr_t& robot () const;

      /// Get pointer to problem
      ProblemPtr_t problem ()
      {
	return problem_;
      }
      /// Get shared pointer to initial configuration.
      const ConfigurationPtr_t& initConfig () const
      {
	return initConf_;
      }
      /// Set initial configuration.
      void initConfig (const ConfigurationPtr_t& config);
      /// Get number of goal configuration.
      const Configurations_t& goalConfigs () const;
      /// Add goal configuration.
      void addGoalConfig (const ConfigurationPtr_t& config);
      /// Reset the set of goal configurations
      void resetGoalConfigs ();
      /// Set path planner type
      void pathPlannerType (const std::string& type);
      /// Get path planner
      const PathPlannerPtr_t& pathPlanner () const
      {
	return pathPlanner_;
      }

      /// Set path optimizer type
      void pathOptimizerType (const std::string& type);
      /// Get path optimizer
      const PathOptimizerPtr_t& pathOptimizer () const
      {
	return pathOptimizer_;
      }

      const RoadmapPtr_t& roadmap () const
      {
	return roadmap_;
      }

      /// Add a constraint
      void addConstraint (const ConstraintPtr_t& constraint);

      /// Get constraint set
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }

      /// Reset constraint set
      void resetConstraints ();

      /// Create new problem.
      void resetProblem ();

      /// \name Solve problem and get paths
      /// \{

      /// Set and solve the problem
      void solve ();

      /// Add a path
      void addPath (const PathVectorPtr_t& path)
      {
	paths_.push_back (path);
      }

      /// Return vector of paths
      const PathVectors_t& paths () const
      {
	return paths_;
      }
      /// \}

      /// \name Obstacles
      /// \{

      /// Add obstacle to the list.
      /// \param inObject a new object.
      /// \param collision whether collision checking should be performed
      ///        for this object.
      /// \param distance whether distance computation should be performed
      ///        for this object.
      void addObstacle (const CollisionObjectPtr_t& inObject, bool collision,
			bool distance);
      /// \}

    private:
      typedef boost::function < PathPlannerPtr_t (const Problem&,
						  const RoadmapPtr_t&) >
	PathPlannerBuilder_t;
      typedef boost::function < PathOptimizerPtr_t (const Problem&) >
	PathOptimizerBuilder_t;
      typedef std::map < std::string, PathPlannerBuilder_t >
	PathPlannerFactory_t;
      ///Map (string , constructor of path optimizer)
      typedef std::map < std::string, PathOptimizerBuilder_t >
	PathOptimizerFactory_t;
      /// Robot
      DevicePtr_t robot_;
      bool robotChanged_;
      /// Problem
      ProblemPtr_t problem_;
      /// Shared pointer to initial configuration.
      ConfigurationPtr_t initConf_;
      /// Shared pointer to goal configuration.
      Configurations_t goalConfigurations_;
      /// Path planner
      std::string pathPlannerType_;
      PathPlannerPtr_t pathPlanner_;
      /// Path optimizer
      std::string pathOptimizerType_;
      PathOptimizerPtr_t pathOptimizer_;
      /// Store roadmap
      RoadmapPtr_t roadmap_;
      /// Paths
      PathVectors_t paths_;
      /// Path planner factory
      PathPlannerFactory_t pathPlannerFactory_;
      /// Path optimizer factory
      PathOptimizerFactory_t pathOptimizerFactory_;
      /// Store constraints until call to solve.
      ConstraintSetPtr_t constraints_;
      /// Store obstacles until call to solve.
      ObjectVector_t collisionObstacles_;
      ObjectVector_t distanceObstacles_;
    }; // class ProblemSolver
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PROBLEM_SOLVER_HH
