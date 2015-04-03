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

# include <stdexcept>

# include <hpp/model/fwd.hh>
# include <boost/function.hpp>
# include <hpp/core/deprecated.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    /// Set and solve a path planning problem
    ///
    /// This class is a container that does the interface between
    /// hpp-core library and component to be running in a middleware
    /// like CORBA or ROS.
    class HPP_CORE_DLLAPI ProblemSolver {
    public:
      typedef boost::function < PathPlannerPtr_t (const Problem&,
						  const RoadmapPtr_t&) >
	PathPlannerBuilder_t;
      typedef boost::function < PathOptimizerPtr_t (const Problem&) >
	PathOptimizerBuilder_t;
      typedef boost::function < PathValidationPtr_t (const DevicePtr_t&,
						     const value_type&) >
	PathValidationBuilder_t;
      typedef boost::function <PathProjectorPtr_t (const DistancePtr_t&,
						   const SteeringMethodPtr_t&,
						   value_type) >
	PathProjectorBuilder_t;
      typedef boost::function <ConfigurationShooterPtr_t (const DevicePtr_t&) >
	ConfigurationShooterBuilder_t;

      typedef std::vector <PathOptimizerPtr_t> PathOptimizers_t;
      typedef std::vector <std::string> PathOptimizerTypes_t;

      /// Create instance and return pointer
      static ProblemSolverPtr_t create ();

      /// Return latest instance created by method create.
      static ProblemSolverPtr_t latest ();

      /// Destructor
      virtual ~ProblemSolver ();

      /// Set robot
      virtual void robot (const DevicePtr_t& robot);

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
      virtual void initConfig (const ConfigurationPtr_t& config);
      /// Get number of goal configuration.
      const Configurations_t& goalConfigs () const;
      /// Add goal configuration.
      virtual void addGoalConfig (const ConfigurationPtr_t& config);
      /// Reset the set of goal configurations
      void resetGoalConfigs ();
      /// Set path planner type
      virtual void pathPlannerType (const std::string& type);
      /// Set configuration shooter type
      void configurationShooterType (const std::string& type);
      /// Add a ConfigurationShooter type
      /// \param type name of the ConfigurationShooter type
      /// \param static method that creates a ConfigurationShooter
      /// with robot as input
      void addConfigurationShooterType (const std::string& type,
			       const ConfigurationShooterBuilder_t& builder)
      {
	configurationShooterFactory_ [type] = builder;
      }
      /// Add a path planner type
      /// \param type name of the new path planner type
      /// \param static method that creates a path planner with a problem
      /// and a roadmap as input
      void addPathPlannerType (const std::string& type,
			       const PathPlannerBuilder_t& builder)
      {
	pathPlannerFactory_ [type] = builder;
      }
      /// Get path planner
      const PathPlannerPtr_t& pathPlanner () const
      {
	return pathPlanner_;
      }

      /// Add a path optimizer in the vector
      ///
      /// \param name of the type of path optimizer that should be added
      void addPathOptimizer (const std::string& type);
      /// Clear the vector of path optimizers
      void clearPathOptimizers ();
      /// Get path optimizer at given rank
      const PathOptimizerPtr_t& pathOptimizer (std::size_t rank) const
      {
	return pathOptimizers_ [rank];
      }
      /// Add a path optimizer type
      /// \param type name of the new path optimizer type
      /// \param static method that creates a path optimizer with a problem
      /// as input
      void addPathOptimizerType (const std::string& type,
				 const PathOptimizerBuilder_t& builder)
      {
	pathOptimizerFactory_ [type] = builder;
      }

      /// Optimize path
      ///
      /// \param path path to optimize
      /// Build vector of path optimizers if needed
      /// \note each intermediate optimization output is stored in this object.
      void optimizePath (PathVectorPtr_t path);

      /// Set path validation method
      /// \param type name of new path validation method
      /// \param tolerance acceptable penetration for path validation
      /// Path validation methods are used to validate edges in path planning
      /// path optimization methods.
      void pathValidationType (const std::string& type,
			       const value_type& tolerance);

      /// Add a path validation type
      /// \param type name of the new path validation method,
      /// \param static method that creates a path validation with a robot
      /// and tolerance as input.
      void addPathValidationType (const std::string& type,
				 const PathValidationBuilder_t& builder)
      {
	pathValidationFactory_ [type] = builder;
      }

      /// Set path projector method
      /// \param type name of new path validation method
      /// \param step discontinuity tolerance
      void pathProjectorType (const std::string& type,
			      const value_type& step);

      /// Add a path projector type
      /// \param type name of the new path projector method,
      /// \param static method that creates a path projector with a distance
      /// and tolerance as input.
      void addPathProjectorType (const std::string& type,
				 const PathProjectorBuilder_t& builder)
      {
	pathProjectorFactory_ [type] = builder;
      }

      const RoadmapPtr_t& roadmap () const
      {
	return roadmap_;
      }

      /// \name Constraints
      /// \{

      /// Add a constraint
      void addConstraint (const ConstraintPtr_t& constraint);

      /// Add a LockedJoint
      void addLockedJoint (const LockedJointPtr_t& lockedJoint);

      /// Get constraint set
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }

      /// Reset constraint set
      virtual void resetConstraints ();

      /// Add a CenterOfMassComputation object to the local map
      /// \param name key of the object in the map
      /// \param comc the object to insert.
      void addCenterOfMassComputation (const std::string& name,
          CenterOfMassComputationPtr_t comc)
      {
        comcMap_ [name] = comc;
      }

      /// Get the CenterOfMassComputation object from the local map
      /// \param name key of the object in the map
      /// \return the corresponding object.
      /// \note a null shared pointer is returned if the object was not found
      CenterOfMassComputationPtr_t centerOfMassComputation (const std::string& name) const
      {
        CenterOfMassComputationMap_t::const_iterator it =
          comcMap_.find (name);
        if (it == comcMap_.end ())
          return CenterOfMassComputationPtr_t ();
        return it->second;
      }

      /// Add differential function to the config projector
      /// \param constraintName Name given to config projector if created by
      ///        this method.
      /// \param functionName name of the function as stored in internal map.
      /// Build the config projector if not yet constructed.
      virtual void addFunctionToConfigProjector
	(const std::string& constraintName, const std::string& functionName);

      /// Add a a numerical constraint in local map.
      /// \param name name of the numerical constraint as stored in local map,
      /// \param constraint numerical constraint
      ///
      /// Numerical constraints are to be inserted in the ConfigProjector of
      /// the constraint set.
      void addNumericalConstraint (const std::string& name,
				   const DifferentiableFunctionPtr_t&
				   constraint)
      {
	numericalConstraintMap_ [name] = constraint;
        comparisonTypeMap_ [name] = ComparisonType::createDefault();
      }

      /// Add a vector of passive dofs in a local map.
      /// \param name the key of the vector in the map.
      /// \param passiveDofs a vector of SizeInterval_t interpreted as
      ///                    (index_start, interval_length).
      void addPassiveDofs (const std::string& name,
                           const SizeIntervals_t& passiveDofs)
      {
        passiveDofsMap_ [name] = passiveDofs;
      }

      /// Get the vector of passive dofs associated with this name.
      SizeIntervals_t passiveDofs (const std::string& name) const
      {
        SizeIntervalsMap_t::const_iterator it = passiveDofsMap_.find (name);
        if (it == passiveDofsMap_.end ())
          return SizeIntervals_t ();
        return it->second;
      }

      /// Set the comparison types of a constraint.
      /// \param name name of the differentiable function.
      void comparisonType (const std::string& name,
			   const ComparisonType::VectorOfTypes types)
      {
        DifferentiableFunctionPtr_t df = numericalConstraintMap_ [name];
        if (!df)
          throw std::logic_error (std::string ("Numerical constraint ") +
				  name + std::string (" not defined."));
        ComparisonTypesPtr_t eqtypes = ComparisonTypes::create (types);
        comparisonTypeMap_ [name] = eqtypes;
      }

      /// Set the comparison type of a constraint
      /// \param name name of the differentiable function.
      void comparisonType (const std::string& name,
			   const ComparisonTypePtr_t eq)
      {
        comparisonTypeMap_ [name] = eq;
      }

      ComparisonTypePtr_t comparisonType (const std::string& name) const
      {
        ComparisonTypeMap_t::const_iterator it = comparisonTypeMap_.find (name);
        if (it == comparisonTypeMap_.end ())
          return Equality::create ();
        return it->second;
      }

      /// Get constraint with given name
      DifferentiableFunctionPtr_t numericalConstraint (const std::string& name)
      {
	return numericalConstraintMap_ [name];
      }

      /// Compute value and Jacobian of numerical constraints
      ///
      /// \param configuration input configuration
      /// \retval value values of the numerical constraints stacked in a unique
      ///         vector,
      /// \retval jacobian Jacobian of the numerical constraints stacked in a
      ///         unique matrix.
      ///
      /// Columns of the Jacobian corresponding to locked joints are omitted,
      /// columns corresponding to passive dofs are set to 0.
      void computeValueAndJacobian (const Configuration_t& configuration,
				    vector_t& value, matrix_t& jacobian) const;

      /// Set maximal number of iterations in config projector
      void maxIterations (size_type iterations)
      {
	maxIterations_ = iterations;
	if (constraints_ && constraints_->configProjector ()) {
	  constraints_->configProjector ()->maxIterations (iterations);
	}
      }
      /// Get maximal number of iterations in config projector
      size_type maxIterations () const
      {
	return maxIterations_;
      }

      /// Set error threshold in config projector
      void errorThreshold (const value_type& threshold)
      {
	errorThreshold_ = threshold;
	if (constraints_ && constraints_->configProjector ()) {
	  constraints_->configProjector ()->errorThreshold (threshold);
	}
      }
      /// Get errorimal number of threshold in config projector
      value_type errorThreshold () const
      {
	return errorThreshold_;
      }
      /// \}

      /// Create new problem.
      virtual void resetProblem ();

      /// Reset the roadmap.
      /// \note When joints bounds are changed, the roadmap must be reset
      ///       because the kd tree must be resized.
      virtual void resetRoadmap ();

      /// \name Solve problem and get paths
      /// \{

      /// Create Path optimizer if needed
      ///
      /// If a path optimizer is already set, do nothing.
      /// Type of optimizer is determined by method selectPathOptimizer.
      void createPathOptimizers ();

      /// Prepare the solver for a step by step planning.
      /// and try to make direct connections (call PathPlanner::tryDirectPath)
      /// \return the return value of PathPlanner::pathExists
      virtual bool prepareSolveStepByStep ();

      /// Execute one step of the planner.
      /// \return the return value of PathPlanner::pathExists of the selected planner.
      /// \note This won't check if a solution has been found before doing one step.
      /// The decision to stop planning is let to the user.
      virtual bool executeOneStep ();

      /// Finish the solving procedure
      /// The path optimizer is not called
      virtual void finishSolveStepByStep ();

      /// Set and solve the problem
      virtual void solve ();

      /// Interrupt path planning and path optimization
      void interrupt ();

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
      virtual void addObstacle (const CollisionObjectPtr_t& inObject, bool collision,
			bool distance);

      /// Remove collision pair between a joint and an obstacle
      /// \param jointName name of the joint,
      /// \param obstacleName name of the obstacle
      void removeObstacleFromJoint (const std::string& jointName,
				    const std::string& obstacleName);

      /// Get obstacle by name
      const CollisionObjectPtr_t& obstacle (const std::string& name);

      /// Get list of obstacle names
      ///
      /// \param collision whether to return collision obstacle names
      /// \param distance whether to return distance obstacle names
      /// \return list of obstacle names
      std::list <std::string> obstacleNames (bool collision, bool distance)
	const;

      /// Return list of pair of distance computations
      const DistanceBetweenObjectsPtr_t& distanceBetweenObjects () const
      {
	return distanceBetweenObjects_;
      }
      /// \}

      /// Local vector of objects considered for collision detection
      const ObjectVector_t& collisionObstacles () const;
      /// Local vector of objects considered for distance computation
      const ObjectVector_t& distanceObstacles () const;

      /// Set the roadmap
      void roadmap (const RoadmapPtr_t& roadmap)
      {
	roadmap_ = roadmap;
      }

    protected:
      /// Constructor
      ///
      /// Call create to create an instance. The latest created instance can
      /// be retrieved by method latest.
      ProblemSolver ();

      /// Store constraints until call to solve.
      ConstraintSetPtr_t constraints_;

      /// Set pointer to problem
      void problem (ProblemPtr_t problem)
      {
        if (problem_)
          delete problem_;
	problem_ = problem;
      }

      /// Initialize the new problem
      /// \param problem is inserted in the ProblemSolver and initialized.
      /// \note The previous Problem, if any, is not deleted. The function
      ///       should be called when creating Problem object, in resetProblem()
      ///       and all reimplementation in inherited class.
      virtual void initializeProblem (ProblemPtr_t problem);

      /// Robot
      DevicePtr_t robot_;
      /// Problem
      ProblemPtr_t problem_;

      PathPlannerPtr_t pathPlanner_;
      /// Store roadmap
      RoadmapPtr_t roadmap_;
      /// Paths
      PathVectors_t paths_;
      /// Path projector method
      std::string pathProjectorType_;
      /// Tolerance of path projector
      value_type pathProjectorTolerance_;
      typedef std::map <std::string, PathProjectorBuilder_t >
        PathProjectorFactory_t;
      /// Path projector factory
      PathProjectorFactory_t pathProjectorFactory_;

      /// Path planner
      std::string pathPlannerType_;
    private:
      /// Map (string , constructor of path planner)
      typedef std::map < std::string, PathPlannerBuilder_t >
	PathPlannerFactory_t;
      /// Map (string , constructor of path optimizer)
      typedef std::map < std::string, PathOptimizerBuilder_t >
	PathOptimizerFactory_t;
      /// Map (string , constructor of path validation method)
      typedef std::map <std::string, PathValidationBuilder_t >
	PathValidationFactory_t;
      /// Map (string , constructor of configuration shooter method)
      typedef std::map <std::string, ConfigurationShooterBuilder_t >
        ConfigurationShooterFactory_t;

      /// Shared pointer to initial configuration.
      ConfigurationPtr_t initConf_;
      /// Shared pointer to goal configuration.
      Configurations_t goalConfigurations_;
      /// Configuration shooter
      std::string configurationShooterType_;
      /// Path optimizer
      PathOptimizerTypes_t pathOptimizerTypes_;
      PathOptimizers_t pathOptimizers_;
      /// Path validation method
      std::string pathValidationType_;
      /// Tolerance of path validation
      value_type pathValidationTolerance_;
      /// Path planner factory
      PathPlannerFactory_t pathPlannerFactory_;
      /// Configuration shooter factory
      ConfigurationShooterFactory_t configurationShooterFactory_;
      /// Path optimizer factory
      PathOptimizerFactory_t pathOptimizerFactory_;
      /// Path validation factory
      PathValidationFactory_t pathValidationFactory_;
      /// Store obstacles until call to solve.
      ObjectVector_t collisionObstacles_;
      ObjectVector_t distanceObstacles_;
      /// Map of obstacles by names
      std::map <std::string, CollisionObjectPtr_t> obstacleMap_;
      // Tolerance for numerical constraint resolution
      value_type errorThreshold_;
      // Maximal number of iterations for numerical constraint resolution
      size_type maxIterations_;
      /// Map of constraints
      DifferentiableFunctionMap_t numericalConstraintMap_;
      /// Map of passive dofs
      SizeIntervalsMap_t passiveDofsMap_;
      /// Map of inequality
      ComparisonTypeMap_t comparisonTypeMap_;
      /// Map of CenterOfMassComputation
      CenterOfMassComputationMap_t comcMap_;
      /// Computation of distances to obstacles
      DistanceBetweenObjectsPtr_t distanceBetweenObjects_;
      /// Store latest instance created by static method create
      static ProblemSolverPtr_t latest_;
    }; // class ProblemSolver
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PROBLEM_SOLVER_HH
