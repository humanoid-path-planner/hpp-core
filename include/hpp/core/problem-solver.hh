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
# include <boost/function.hpp>

# include <hpp/pinocchio/fwd.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/config-projector.hh>
# include <hpp/core/deprecated.hh>
# include <hpp/core/container.hh>
# include <hpp/core/constraint-set.hh>
# include <hpp/core/comparison-type.hh> // ComparisonType::VectorOfTypes

namespace hpp {
  namespace core {
    typedef boost::function < PathOptimizerPtr_t (const Problem&) >
      PathOptimizerBuilder_t;
    typedef boost::function < PathPlannerPtr_t (const Problem&,
        const RoadmapPtr_t&) >
      PathPlannerBuilder_t;
    typedef boost::function < PathValidationPtr_t (const DevicePtr_t&,
        const value_type&) >
      PathValidationBuilder_t;
    typedef boost::function <PathProjectorPtr_t (const Problem&,
        const value_type&) >
      PathProjectorBuilder_t;
    typedef boost::function <ConfigurationShooterPtr_t (const DevicePtr_t&) >
      ConfigurationShooterBuilder_t;
  typedef boost::function <DistancePtr_t (const Problem&) >
    DistanceBuilder_t;
    typedef boost::function <SteeringMethodPtr_t (const Problem&) >
      SteeringMethodBuilder_t;
    typedef std::vector<CollisionObjectPtr_t > AffordanceObjects_t;
    typedef vector3_t AffordanceConfig_t;

    /// Set and solve a path planning problem
    ///
    /// This class is a container that does the interface between
    /// hpp-core library and component to be running in a middleware
    /// like CORBA or ROS.
    class HPP_CORE_DLLAPI ProblemSolver :
      public Containers <
        boost::mpl::vector < PathPlannerBuilder_t,
                             PathOptimizerBuilder_t,
                             PathValidationBuilder_t,
                             PathProjectorBuilder_t,
                             ConfigurationShooterBuilder_t,
                             NumericalConstraintPtr_t,
                             LockedJointPtr_t,
                             DistanceBuilder_t,
                             SteeringMethodBuilder_t,
                             AffordanceObjects_t,
                             AffordanceConfig_t,
                             JointAndShapes_t> >
    {
    public:

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
      /*
      /// Add goal constraints
      void addGoalConstraint (const ConstraintPtr_t& constraint);
      /// Add goal LockedJoint
      void addGoalConstraint (const LockedJointPtr_t& lj);
      /// Add goal numerical constraints
      void addGoalConstraint (const std::string& constraintName,
          const std::string& functionName, const std::size_t priority);
      /// Reset goal constraints
      void resetGoalConstraint ();
      */
      /// Set path planner type
      virtual void pathPlannerType (const std::string& type);
      const std::string& pathPlannerType () const {
        return pathPlannerType_;
      }
      /// Set distance type
      void distanceType (const std::string& type);
      const std::string& distanceType () const {
        return distanceType_;
      }
      /// Set steering method type
      void steeringMethodType (const std::string& type);
      const std::string& steeringMethodType () const {
        return steeringMethodType_;
      }
      /// Add a distance type
      /// \param type name of the distance type
      /// \param static method that creates a distance
      /// with robot as input
      /// \deprecated use add <DistanceBuilder_t> (type, builder) instead
      void addDistanceType (const std::string& type,
                   const DistanceBuilder_t& builder) HPP_CORE_DEPRECATED
      {
    add <DistanceBuilder_t> (type, builder);
      }
      /// Add a SteeringMethod type
      /// \param type name of the SteeringMethod type
      /// \param static method that creates a SteeringMethod
      /// with robot as input
      void addSteeringMethodType (const std::string& type,
			       const SteeringMethodBuilder_t& builder)
      {
	add <SteeringMethodBuilder_t> (type, builder);
      }
      /// Set configuration shooter type
      void configurationShooterType (const std::string& type);
      const std::string& configurationShooterType () const {
        return configurationShooterType_;
      }
      /// Add a ConfigurationShooter type
      /// \param type name of the ConfigurationShooter type
      /// \param static method that creates a ConfigurationShooter
      /// with robot as input
      /// \deprecated use add <ConfigurationShooterBuilder_t> (type, builder)
      /// instead.
      void addConfigurationShooterType (const std::string& type,
			       const ConfigurationShooterBuilder_t& builder)
        HPP_CORE_DEPRECATED
      {
	add <ConfigurationShooterBuilder_t> (type, builder);
      }
      /// Add a path planner type
      /// \param type name of the new path planner type
      /// \param static method that creates a path planner with a problem
      /// and a roadmap as input
      /// \deprecated use add <PathPlannerBuilder_t> (type, builder) instead
      void addPathPlannerType (const std::string& type,
			       const PathPlannerBuilder_t& builder)
        HPP_CORE_DEPRECATED
      {
	add <PathPlannerBuilder_t> (type, builder);
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
      PathOptimizerTypes_t pathOptimizerTypes () const {
        return pathOptimizerTypes_;
      }
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
      /// \deprecated use add <PathOptimizerBuilder_t> (type, builder) instead.
      void addPathOptimizerType (const std::string& type,
				 const PathOptimizerBuilder_t& builder)
        HPP_CORE_DEPRECATED
      {
        add (type, builder);
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
      virtual void pathValidationType (const std::string& type,
                                       const value_type& tolerance);
      const std::string& pathValidationType (value_type& tolerance) {
        tolerance = pathValidationTolerance_;
        return pathValidationType_;
      }

      /// Add a path validation type
      /// \param type name of the new path validation method,
      /// \param static method that creates a path validation with a robot
      /// and tolerance as input.
      /// \deprecated use add <PathValidationBuilder_t> (type, builder) instead.
      void addPathValidationType (const std::string& type,
				 const PathValidationBuilder_t& builder)
        HPP_CORE_DEPRECATED
      {
	add (type, builder);
      }

      /// Set path projector method
      /// \param type name of new path validation method
      /// \param step discontinuity tolerance
      void pathProjectorType (const std::string& type,
			      const value_type& step);

      /// Get path projector current type and get tolerance
      const std::string& pathProjectorType (value_type& tolerance) const {
        tolerance = pathProjectorTolerance_;
        return pathProjectorType_;
      }

      /// Add a path projector type
      /// \param type name of the new path projector method,
      /// \param static method that creates a path projector with a distance
      /// and tolerance as input.
      /// \deprecated use add <PathProjectorBuilder_t> (type, builder) instead.
      void addPathProjectorType (const std::string& type,
				 const PathProjectorBuilder_t& builder)
        HPP_CORE_DEPRECATED
      {
	add (type, builder);
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
      /// \deprecated use createLockedJoint and addLockedJointToConfigProjector
      ///             instead
      void addLockedJoint (const LockedJointPtr_t& lockedJoint)
        HPP_CORE_DEPRECATED;

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

      /// Add numerical constraint to the config projector
      /// \param configProjName Name given to config projector if created by
      ///        this method.
      /// \param  constraintName name of the function as stored in internal map.
      /// Build the config projector if not yet constructed.
      virtual void addNumericalConstraintToConfigProjector
	(const std::string& configProjName, const std::string& constraintName,
         const std::size_t priority = 0);

      /// Add locked joint to the config projector
      /// \param configProjName Name given to config projector if created by
      ///        this method.
      /// \param lockedJointName name of the locked joint as stored in internal
      ///        map.
      /// Build the config projector if not yet constructed.
      virtual void addLockedJointToConfigProjector
	(const std::string& configProjName, const std::string& lockedJointName);

      /// Add a a numerical constraint in local map.
      /// \param name name of the numerical constraint as stored in local map,
      /// \param constraint numerical constraint
      ///
      /// Numerical constraints are to be inserted in the ConfigProjector of
      /// the constraint set.
      void addNumericalConstraint (const std::string& name,
				   const NumericalConstraintPtr_t&
				   constraint)
      {
        add (name, constraint);
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
			   const ComparisonType::VectorOfTypes types);

      void comparisonType (const std::string& name,
         const ComparisonType::Type &type);

      /// Set the comparison type of a constraint
      /// \param name name of the differentiable function.
      void comparisonType (const std::string& name,
			   const ComparisonTypePtr_t eq);

      ComparisonTypePtr_t comparisonType (const std::string& name) const;

      /// Get constraint with given name
      NumericalConstraintPtr_t numericalConstraint (const std::string& name)
      {
        if (!has <NumericalConstraintPtr_t> (name))
          return NumericalConstraintPtr_t ();
        return get <NumericalConstraintPtr_t> (name);
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
      void maxIterProjection (size_type iterations)
      {
	maxIterProjection_ = iterations;
	if (constraints_ && constraints_->configProjector ()) {
	  constraints_->configProjector ()->maxIterations (iterations);
	}
      }
      /// Get maximal number of iterations in config projector
      size_type maxIterProjection () const
      {
	return maxIterProjection_;
      }

      /// Set maximal number of iterations in config projector
      void maxIterPathPlanning (size_type iterations)
      {
	maxIterPathPlanning_ = iterations;
	if (constraints_ && constraints_->configProjector ()) {
	  constraints_->configProjector ()->maxIterations (iterations);
	}
      }
      /// Get maximal number of iterations in config projector
      size_type maxIterPathPlanning () const
      {
	return maxIterPathPlanning_;
      }

      /// Set error threshold in config projector
      void errorThreshold (const value_type& threshold);
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

      /// Make direct connection between two configurations
      /// \param start, end: the configurations to link.
      /// \param validate whether path should be validated. If true, path
      ///        validation is called and only valid part of path is inserted
      ///        in the path vector.
      /// \retval pathId Id of the path that is inserted in the path vector,
      /// \retval report Reason for non validation if relevant.
      /// return false if direct path is not fully valid
      ///
      /// \note If path is only partly valid, valid part starting at start
      ///       configuration is inserted in path vector.
      bool directPath (ConfigurationIn_t start, ConfigurationIn_t end,
		       bool validate, std::size_t& pathId, std::string& report);

      /// Add random configuration into roadmap as new node.
      void addConfigToRoadmap (const ConfigurationPtr_t& config);

      /// Add an edge between two roadmap nodes.
      ///
      /// \param config1 configuration of start node,
      /// \param config2 configuration of destination node,
      /// \param path path to store in the edge.
      ///
      /// Check that nodes containing config1 and config2 exist in the roadmap.
      void addEdgeToRoadmap (const ConfigurationPtr_t& config1,
			     const ConfigurationPtr_t& config2,
			     const PathPtr_t& path);

      /// Interrupt path planning and path optimization
      void interrupt ();

      /// Add a path
      std::size_t addPath (const PathVectorPtr_t& path)
      {
        std::size_t s = paths_.size();
	paths_.push_back (path);
        return s;
      }

      /// Erase a path.
      void erasePath (std::size_t pathId)
      {
	PathVectors_t::iterator it = paths_.begin();
	std::advance(it, pathId);

	paths_.erase(it);
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
      virtual void addObstacle (const CollisionObjectPtr_t &inObject, bool collision,
            bool distance);

      /// Remove obstacle from the list.
      /// \param name name of the obstacle
      virtual void removeObstacle (const std::string& name);

      /// Add obstacle to the list.
      /// \param inObject a new object.
      /// \param collision whether collision checking should be performed
      ///        for this object.
      /// \param distance whether distance computation should be performed
      ///        for this object.
      virtual void addObstacle (const std::string& name,
          /*const*/ FclCollisionObject &inObject,
          bool collision,
          bool distance);

      /// Remove collision pair between a joint and an obstacle
      /// \param jointName name of the joint,
      /// \param obstacleName name of the obstacle
      void removeObstacleFromJoint (const std::string& jointName,
				    const std::string& obstacleName);

      /// Extract from the obstacle the part that can collide with aabb
      /// \warning the obstacle is removed if there are not possible collision.
      void cutObstacle (const std::string& name, const fcl::AABB& aabb);

      /// Build matrix of relative motions between joints
      ///
      /// Call Problem::filterCollisionPairs.
      void filterCollisionPairs ();

      /// Get obstacle by name
      /// Throws if obstacle does not exists.
      CollisionObjectPtr_t obstacle (const std::string& name) const;

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
      const ObjectStdVector_t& collisionObstacles () const;
      /// Local vector of objects considered for distance computation
      const ObjectStdVector_t& distanceObstacles () const;

      /// Set the roadmap
      void roadmap (const RoadmapPtr_t& roadmap)
      {
	roadmap_ = roadmap;
      }

      /// Initialize distance
      ///
      /// Set distance by calling the distance factory
      void initDistance ();

      /// Initialize steering method
      ///
      /// Set steering method by calling the steering method factory
      void initSteeringMethod ();

      /// Initialize path projector
      ///
      /// Set path projector by calling path projector factory
      void initPathProjector ();

      /// Set path validation by calling path validation factory
      void initPathValidation ();

      /// Initialize the problem target by calling the path validation factory
      virtual void initProblemTarget ();

    protected:
      /// Constructor
      ///
      /// Call create to create an instance. The latest created instance can
      /// be retrieved by method latest.
      ProblemSolver ();

      /// Store constraints until call to solve.
      ConstraintSetPtr_t constraints_;

      /// Set pointer to problem
      void problem (ProblemPtr_t problem);

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

      /// Path planner
      std::string pathPlannerType_;

      /// Shared pointer to the problem target
      ProblemTargetPtr_t target_;
    private:
      /// Shared pointer to initial configuration.
      ConfigurationPtr_t initConf_;
      /// Shared pointer to goal configuration.
      Configurations_t goalConfigurations_;
      /// Configuration shooter
      std::string configurationShooterType_;
      /// Steering method
      std::string distanceType_;
      /// Steering method
      std::string steeringMethodType_;
      /// Path optimizer
      PathOptimizerTypes_t pathOptimizerTypes_;
      PathOptimizers_t pathOptimizers_;
      /// Path validation method
      std::string pathValidationType_;
      /// Tolerance of path validation
      value_type pathValidationTolerance_;
      /// Store obstacles until call to solve.
      ObjectStdVector_t collisionObstacles_; // FIXME should be removed?
      ObjectStdVector_t distanceObstacles_;  // FIXME should be removed?
      pinocchio::GeomModelPtr_t obstacleModel_;
      pinocchio::GeomDataPtr_t  obstacleData_;
      // Tolerance for numerical constraint resolution
      value_type errorThreshold_;
      // Maximal number of iterations for numerical constraint resolution
      size_type maxIterProjection_;
      /// Maximal number of iterations for path planner
      unsigned long int maxIterPathPlanning_;
      /// Map of passive dofs
      SizeIntervalsMap_t passiveDofsMap_;
      /// Map of CenterOfMassComputation
      CenterOfMassComputationMap_t comcMap_;
      /// Computation of distances to obstacles
      DistanceBetweenObjectsPtr_t distanceBetweenObjects_;
      /// Store latest instance created by static method create
      static ProblemSolverPtr_t latest_;

      void initProblem ();
    }; // class ProblemSolver
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PROBLEM_SOLVER_HH
