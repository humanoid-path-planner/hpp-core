//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_PROBLEM_SOLVER_HH
#define HPP_CORE_PROBLEM_SOLVER_HH

#include <functional>
#include <hpp/core/config.hh>
#include <hpp/core/container.hh>
#include <hpp/core/deprecated.hh>
#include <hpp/core/fwd.hh>
#include <hpp/pinocchio/fwd.hh>
#include <stdexcept>

namespace hpp {
namespace core {
/// member ProblemSolver::lockedJoints has been removed. LockedJointPtr_t
/// instances are now stored with constraints::ImplicitPtr_t in
/// member numericalConstraints.
class
    Member_lockedJoints_in_class_ProblemSolver_has_been_removed_use_member_numericalConstraints_instead {
};
typedef std::function<DevicePtr_t(const std::string&)> RobotBuilder_t;
typedef std::function<PathOptimizerPtr_t(const ProblemConstPtr_t&)>
    PathOptimizerBuilder_t;
typedef std::function<PathPlannerPtr_t(const ProblemConstPtr_t&,
                                       const RoadmapPtr_t&)>
    PathPlannerBuilder_t;
typedef std::function<PathValidationPtr_t(const DevicePtr_t&,
                                          const value_type&)>
    PathValidationBuilder_t;
typedef std::function<ConfigValidationPtr_t(const DevicePtr_t&)>
    ConfigValidationBuilder_t;
typedef std::function<PathProjectorPtr_t(const ProblemConstPtr_t&,
                                         const value_type&)>
    PathProjectorBuilder_t;
typedef std::function<ConfigurationShooterPtr_t(const ProblemConstPtr_t&)>
    ConfigurationShooterBuilder_t;
typedef std::function<DistancePtr_t(const ProblemConstPtr_t&)>
    DistanceBuilder_t;
typedef std::function<SteeringMethodPtr_t(const ProblemConstPtr_t&)>
    SteeringMethodBuilder_t;
typedef std::vector<std::pair<std::string, CollisionObjectPtr_t> >
    AffordanceObjects_t;
typedef vector3_t AffordanceConfig_t;

/// Set and solve a path planning problem
///
/// This class is a container that does the interface between
/// hpp-core library and component to be running in a middleware
/// like CORBA or ROS.
class HPP_CORE_DLLAPI ProblemSolver {
 public:
  typedef std::vector<PathOptimizerPtr_t> PathOptimizers_t;
  typedef std::vector<std::string> PathOptimizerTypes_t;
  typedef std::vector<std::string> ConfigValidationTypes_t;

  /// Create instance and return pointer
  static ProblemSolverPtr_t create();

  /// Destructor
  virtual ~ProblemSolver();

  /// Set robot type
  /// \param type type of the robots that will be created later
  void robotType(const std::string& type);

  /// Get robot type
  const std::string& robotType() const;

  /// Create a robot of a type defined by method setRobotType
  ///
  /// \param name name of the robot
  ///
  /// Robot is stored in problemSolver.
  DevicePtr_t createRobot(const std::string& name);

  /// Set robot
  virtual void robot(const DevicePtr_t& robot);

  /// Get robot
  const DevicePtr_t& robot() const;

  /// Get pointer to problem
  ProblemPtr_t problem() { return problem_; }
  /// Get shared pointer to initial configuration.
  const ConfigurationPtr_t& initConfig() const { return initConf_; }
  /// Set initial configuration.
  virtual void initConfig(const ConfigurationPtr_t& config);
  /// Get number of goal configuration.
  const Configurations_t& goalConfigs() const;
  /// Add goal configuration.
  virtual void addGoalConfig(const ConfigurationPtr_t& config);
  /// Reset the set of goal configurations
  void resetGoalConfigs();
  /// Set goal of path planning as a set of constraints
  void setGoalConstraints(const NumericalConstraints_t& constraints);
  /// Stop defining the goal of path planning as a set of constraints
  void resetGoalConstraints();
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
  virtual void pathPlannerType(const std::string& type);
  const std::string& pathPlannerType() const { return pathPlannerType_; }
  /// Set distance type
  void distanceType(const std::string& type);
  const std::string& distanceType() const { return distanceType_; }
  /// Set steering method type
  void steeringMethodType(const std::string& type);
  const std::string& steeringMethodType() const { return steeringMethodType_; }
  /// Set configuration shooter type
  void configurationShooterType(const std::string& type);
  const std::string& configurationShooterType() const {
    return configurationShooterType_;
  }
  /// Get path planner
  const PathPlannerPtr_t& pathPlanner() const { return pathPlanner_; }

  /// Add a path optimizer in the vector
  ///
  /// \param name of the type of path optimizer that should be added
  void addPathOptimizer(const std::string& type);
  const PathOptimizerTypes_t& pathOptimizerTypes() const {
    return pathOptimizerTypes_;
  }
  /// Clear the vector of path optimizers
  void clearPathOptimizers();
  /// Get path optimizer at given rank
  const PathOptimizerPtr_t& pathOptimizer(std::size_t rank) const {
    return pathOptimizers_[rank];
  }

  /// Optimize path
  ///
  /// \param path path to optimize
  /// Build vector of path optimizers if needed
  /// \note each intermediate optimization output is stored in this object.
  void optimizePath(PathVectorPtr_t path);

  /// Set path validation method
  /// \param type name of new path validation method
  /// \param tolerance acceptable penetration for path validation
  /// Path validation methods are used to validate edges in path planning
  /// path optimization methods.
  virtual void pathValidationType(const std::string& type,
                                  const value_type& tolerance);
  const std::string& pathValidationType(value_type& tolerance) const {
    tolerance = pathValidationTolerance_;
    return pathValidationType_;
  }

  /// Set path projector method
  /// \param type name of new path validation method
  /// \param step discontinuity tolerance
  void pathProjectorType(const std::string& type, const value_type& step);

  /// Get path projector current type and get tolerance
  const std::string& pathProjectorType(value_type& tolerance) const {
    tolerance = pathProjectorTolerance_;
    return pathProjectorType_;
  }

  /// Add a config validation method
  /// \param type name of new config validation method
  /// Config validation methods are used to validate individual
  /// configurations of the robot.
  virtual void addConfigValidation(const std::string& type);

  /// Get config validation current types
  const ConfigValidationTypes_t configValidationTypes() {
    return configValidationTypes_;
  }

  // Clear the vector of config validations
  void clearConfigValidations();

  /// Add a new available config validation method
  void addConfigValidationBuilder(const std::string& type,
                                  const ConfigValidationBuilder_t& builder);

  const RoadmapPtr_t& roadmap() const { return roadmap_; }

  /// \name Constraints
  /// \{

  /// Add a constraint
  void addConstraint(const ConstraintPtr_t& constraint);

  /// Get constraint set
  const ConstraintSetPtr_t& constraints() const { return constraints_; }

  /// Reset constraint set
  virtual void resetConstraints();

  /// Add numerical constraint to the config projector
  /// \param configProjName Name given to config projector if created by
  ///        this method.
  /// \param  constraintName name of the function as stored in internal map.
  /// Build the config projector if not yet constructed.
  virtual void addNumericalConstraintToConfigProjector(
      const std::string& configProjName, const std::string& constraintName,
      const std::size_t priority = 0);

  /// Add a a numerical constraint in local map.
  /// \param name name of the numerical constraint as stored in local map,
  /// \param constraint numerical constraint
  ///
  /// Numerical constraints are to be inserted in the ConfigProjector of
  /// the constraint set.
  void addNumericalConstraint(const std::string& name,
                              const constraints::ImplicitPtr_t& constraint) {
    numericalConstraints.add(name, constraint);
  }

  /// Set the comparison types of a constraint.
  /// \param name name of the differentiable function.
  void comparisonType(const std::string& name, const ComparisonTypes_t types);

  void comparisonType(const std::string& name, const ComparisonType& type);

  ComparisonTypes_t comparisonType(const std::string& name) const;

  /// Get constraint with given name
  constraints::ImplicitPtr_t numericalConstraint(const std::string& name) {
    return numericalConstraints.get(name, constraints::ImplicitPtr_t());
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
  void computeValueAndJacobian(const Configuration_t& configuration,
                               vector_t& value, matrix_t& jacobian) const;

  /// Set maximal number of iterations in config projector
  void maxIterProjection(size_type iterations);
  /// Get maximal number of iterations in config projector
  size_type maxIterProjection() const { return maxIterProjection_; }

  /// Set maximal number of iterations in config projector
  void maxIterPathPlanning(size_type iterations);
  /// Get maximal number of iterations in config projector
  size_type maxIterPathPlanning() const { return maxIterPathPlanning_; }

  /// set time out for the path planning ( in seconds)
  void setTimeOutPathPlanning(double timeOut) {
    timeOutPathPlanning_ = timeOut;
  }

  /// set time out for the path planning ( in seconds)
  double getTimeOutPathPlanning() { return timeOutPathPlanning_; }

  /// Set error threshold in config projector
  void errorThreshold(const value_type& threshold);
  /// Get errorimal number of threshold in config projector
  value_type errorThreshold() const { return errorThreshold_; }
  /// \}

  /// Create new problem.
  virtual void resetProblem();

  /// Reset the roadmap.
  /// \note When joints bounds are changed, the roadmap must be reset
  ///       because the kd tree must be resized.
  virtual void resetRoadmap();

  /// \name Solve problem and get paths
  /// \{

  /// Create Path optimizer if needed
  ///
  /// If a path optimizer is already set, do nothing.
  /// Type of optimizer is determined by method selectPathOptimizer.
  void createPathOptimizers();

  /// Prepare the solver for a step by step planning.
  /// and try to make direct connections (call PathPlanner::tryDirectPath)
  /// \return the return value of PathPlanner::pathExists
  virtual bool prepareSolveStepByStep();

  /// Execute one step of the planner.
  /// \return the return value of PathPlanner::pathExists of the selected
  /// planner. \note This won't check if a solution has been found before doing
  /// one step. The decision to stop planning is let to the user.
  virtual bool executeOneStep();

  /// Finish the solving procedure
  /// The path optimizer is not called
  virtual void finishSolveStepByStep();

  /// Set and solve the problem
  virtual void solve();

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
  bool directPath(ConfigurationIn_t start, ConfigurationIn_t end, bool validate,
                  std::size_t& pathId, std::string& report);

  /// Add random configuration into roadmap as new node.
  void addConfigToRoadmap(const ConfigurationPtr_t& config);

  /// Add an edge between two roadmap nodes.
  ///
  /// \param config1 configuration of start node,
  /// \param config2 configuration of destination node,
  /// \param path path to store in the edge.
  ///
  /// Check that nodes containing config1 and config2 exist in the roadmap.
  void addEdgeToRoadmap(const ConfigurationPtr_t& config1,
                        const ConfigurationPtr_t& config2,
                        const PathPtr_t& path);

  /// Interrupt path planning and path optimization
  void interrupt();

  /// Add a path
  std::size_t addPath(const PathVectorPtr_t& path) {
    std::size_t s = paths_.size();
    paths_.push_back(path);
    return s;
  }

  /// Erase a path.
  void erasePath(std::size_t pathId) {
    PathVectors_t::iterator it = paths_.begin();
    std::advance(it, pathId);

    paths_.erase(it);
  }

  /// Return vector of paths
  const PathVectors_t& paths() const { return paths_; }
  /// \}

  /// \name Obstacles
  /// \{

  /// Add collision objects of a device as obstacles to the list.
  /// \param device the Device to be added.
  /// \param collision whether collision checking should be performed
  ///        for this object.
  /// \param distance whether distance computation should be performed
  ///        for this object.
  virtual void addObstacle(const DevicePtr_t& device, bool collision,
                           bool distance);

  /// Add obstacle to the list.
  /// \param inObject a new object.
  /// \param collision whether collision checking should be performed
  ///        for this object.
  /// \param distance whether distance computation should be performed
  ///        for this object.
  virtual void addObstacle(const CollisionObjectPtr_t& inObject, bool collision,
                           bool distance);

  /// Remove obstacle from the list.
  /// \param name name of the obstacle
  virtual void removeObstacle(const std::string& name);

  /// Add obstacle to the list.
  /// \param inObject a new object.
  /// \param pose the object pose.
  /// \param collision whether collision checking should be performed
  ///        for this object.
  /// \param distance whether distance computation should be performed
  ///        for this object.
  virtual void addObstacle(const std::string& name,
                           const CollisionGeometryPtr_t& inObject,
                           const Transform3f& pose, bool collision,
                           bool distance);

  /// Remove collision pair between a joint and an obstacle
  /// \param jointName name of the joint,
  /// \param obstacleName name of the obstacle
  void removeObstacleFromJoint(const std::string& jointName,
                               const std::string& obstacleName);

  /// Extract from the obstacle the part that can collide with aabb
  /// \warning the obstacle is removed if there are not possible collision.
  void cutObstacle(const std::string& name, const fcl::AABB& aabb);

  /// Build matrix of relative motions between joints
  ///
  /// Call Problem::filterCollisionPairs.
  void filterCollisionPairs();

  /// Get obstacle by name
  /// Throws if obstacle does not exists.
  CollisionObjectPtr_t obstacle(const std::string& name) const;

  /// Get obstacle frame position by name
  /// Throws if obstacle frame does not exists.
  const Transform3f& obstacleFramePosition(const std::string& name) const;

  /// Get list of obstacle names
  ///
  /// \param collision whether to return collision obstacle names
  /// \param distance whether to return distance obstacle names
  /// \return list of obstacle names
  std::list<std::string> obstacleNames(bool collision, bool distance) const;

  /// Return list of pair of distance computations
  const DistanceBetweenObjectsPtr_t& distanceBetweenObjects() const {
    return distanceBetweenObjects_;
  }

  pinocchio::GeomModelPtr_t obstacleGeomModel() const { return obstacleModel_; }
  pinocchio::GeomDataPtr_t obstacleGeomData() const { return obstacleData_; }
  /// \}

  /// Local vector of objects considered for collision detection
  const ObjectStdVector_t& collisionObstacles() const;
  /// Local vector of objects considered for distance computation
  const ObjectStdVector_t& distanceObstacles() const;

  /// Set the roadmap
  void roadmap(const RoadmapPtr_t& roadmap) { roadmap_ = roadmap; }

  /// Initialize distance
  ///
  /// Set distance by calling the distance factory
  void initDistance();

  /// Initialize steering method
  ///
  /// Set steering method by calling the steering method factory
  void initSteeringMethod();

  /// Initialize path projector
  ///
  /// Set path projector by calling path projector factory
  void initPathProjector();

  /// Set path validation by calling path validation factory
  void initPathValidation();

  /// Set config validation by calling config validation factories
  void initConfigValidation();

  /// Initialize the config and path validations and add the obstacles.
  void initValidations();

  /// Initialize the problem target by calling the path validation factory
  virtual void initProblemTarget();

  /// Container of static method that creates a Robot
  /// with string as input
  Container<RobotBuilder_t> robots;
  /// Container of static method that creates a ConfigurationShooter
  /// with a robot as input
  Container<ConfigurationShooterBuilder_t> configurationShooters;
  /// Container of static method that creates a SteeringMethod
  /// with a problem as input
  Container<SteeringMethodBuilder_t> steeringMethods;
  /// Container of static method that creates a Distance
  /// with problem as input
  Container<DistanceBuilder_t> distances;
  /// Container of static method that creates a PathValidation
  /// with a robot and a tolerance as input.
  Container<PathValidationBuilder_t> pathValidations;
  /// Container of static method that creates a ConfigValidation
  /// with a robot as input.
  Container<ConfigValidationBuilder_t> configValidations;
  /// Container of static method that creates a PathProjection
  /// with a problem and a tolerance as input.
  Container<PathProjectorBuilder_t> pathProjectors;
  /// Container of static method that creates a PathPlanner
  /// with a problem and a roadmap as input
  Container<PathPlannerBuilder_t> pathPlanners;
  /// Container of static method that creates a PathOptimizer
  /// with a problem as input
  Container<PathOptimizerBuilder_t> pathOptimizers;

  /// Container of constraints::Implicit
  Container<constraints::ImplicitPtr_t> numericalConstraints;
  /// member lockedJoints has been removed. LockedJointPtr_t
  /// instances are now stored with constraints::ImplicitPtr_t in
  /// member numericalConstraints.
  Member_lockedJoints_in_class_ProblemSolver_has_been_removed_use_member_numericalConstraints_instead
      lockedJoints;
  /// Container of CenterOfMassComputation
  Container<CenterOfMassComputationPtr_t> centerOfMassComputations;
  /// Container of passive DoFs (as segments_t)
  Container<segments_t> passiveDofs;
  /// Container of JointAndShapes_t
  Container<JointAndShapes_t> jointAndShapes;
  /// Container of AffordanceObjects_t
  Container<AffordanceObjects_t> affordanceObjects;
  /// Container of AffordanceConfig_t
  Container<AffordanceConfig_t> affordanceConfigs;

 protected:
  /// Constructor
  ///
  /// Call create to create an instance.
  ProblemSolver();

  /// Store constraints until call to solve.
  ConstraintSetPtr_t constraints_;

  /// Set pointer to problem
  void problem(ProblemPtr_t problem);

  /// Initialize the new problem
  /// \param problem is inserted in the ProblemSolver and initialized.
  /// \note The previous Problem, if any, is not deleted. The function
  ///       should be called when creating Problem object, in resetProblem()
  ///       and all reimplementation in inherited class.
  virtual void initializeProblem(ProblemPtr_t problem);

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
  /// Robot type
  std::string robotType_;
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
  // Config validation methods
  ConfigValidationTypes_t configValidationTypes_;
  /// Store obstacles until call to solve.
  ObjectStdVector_t collisionObstacles_;  // FIXME should be removed?
  ObjectStdVector_t distanceObstacles_;   // FIXME should be removed?
  pinocchio::ModelPtr_t obstacleRModel_;  // Contains the frames
  pinocchio::DataPtr_t obstacleRData_;    // Contains the frames
  pinocchio::GeomModelPtr_t obstacleModel_;
  pinocchio::GeomDataPtr_t obstacleData_;
  // Tolerance for numerical constraint resolution
  value_type errorThreshold_;
  // Maximal number of iterations for numerical constraint resolution
  size_type maxIterProjection_;
  /// Maximal number of iterations for path planner
  unsigned long int maxIterPathPlanning_;
  /// Maximal time allocated to the path-planner
  double timeOutPathPlanning_;
  /// Map of passive dofs
  segmentsMap_t passiveDofsMap_;
  /// Map of CenterOfMassComputation
  CenterOfMassComputationMap_t comcMap_;
  /// Computation of distances to obstacles
  DistanceBetweenObjectsPtr_t distanceBetweenObjects_;
  void initProblem();
};  // class ProblemSolver
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PROBLEM_SOLVER_HH
