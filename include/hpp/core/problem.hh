
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

#ifndef HPP_CORE_PROBLEM_HH
#define HPP_CORE_PROBLEM_HH

#include <hpp/core/config.hh>
#include <hpp/core/container.hh>
#include <hpp/core/parameter.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/pointer.hh>
#include <stdexcept>

namespace hpp {
namespace core {
/// \addtogroup path_planning
/// \{

/// Defines a path planning problem for one robot.
/// A path planning problem is defined by
/// \li a robot: instance of class hpp::pinocchio::Device,
/// \li a set of obstacles: a list of hpp::pinocchio::CollisionObject,
/// \li initial and goal configurations,
/// \li a SteeringMethod to handle the robot dynamics,
/// Additional objects are stored in this object:
/// \li a method to validate paths,
/// \li a set of methods to validate configurations. Default methods are
/// collision checking and joint bound checking.
class HPP_CORE_DLLAPI Problem {
 public:
  /// Create a path planning problem.
  /// \param robot robot associated to the path planning problem.
  static ProblemPtr_t create(DevicePtr_t robot);

  /// Copy a path planning problem and return a shared pointer.
  /// \param other problem to copy
  static ProblemPtr_t createCopy(const ProblemConstPtr_t& other);

  /// Constructor without argument
  /// \warning do not use this constructor. It is necessary to define
  ///          std::pairs <Problem, OtherType>.
  Problem();

  virtual ~Problem();

  /// \name Problem definition
  /// \{

  /// return shared pointer to robot.
  const DevicePtr_t& robot() const { return robot_; }

  /// Get shared pointer to initial configuration.
  const Configuration_t& initConfig() const { return initConf_; }
  /// Set initial configuration.
  void initConfig(ConfigurationIn_t inConfig);
  /// Set the target
  void target(const ProblemTargetPtr_t& target) { target_ = target; }
  /// Get the target
  const ProblemTargetPtr_t& target() const { return target_; }
  /// Get goal configurations.
  /// if problem target is an instance of problemTarget::GoalConfigurations,
  /// return the vector of goal configurations. Otherwise return an empty
  /// vector.
  const Configurations_t goalConfigs() const;
  /// Add goal configuration.
  /// if problem target is not an instance of
  /// problemTarget::GoalConfigurations, set problem target as a new
  /// instance of this class and add the given configuration as a goal.
  void addGoalConfig(ConfigurationIn_t config);
  /// Reset the set of goal configurations
  /// if problem target is not an instance of
  /// problemTarget::GoalConfigurations, set problem target as a new
  /// empty instance of this class.
  void resetGoalConfigs();

  /// \}

  /// \name Steering method and distance function
  /// \{

  /// Set steering method
  /// \param sm steering method.
  /// If problem contains constraints they are passed to the steering
  /// method.
  void steeringMethod(const SteeringMethodPtr_t& sm) {
    steeringMethod_ = sm;
    if (constraints_) steeringMethod_->constraints(constraints_);
  }

  /// Get steering method
  SteeringMethodPtr_t steeringMethod() const { return steeringMethod_; }

  /// Set distance between configurations
  void distance(const DistancePtr_t& distance) { distance_ = distance; }
  /// Get distance between configuration
  const DistancePtr_t& distance() const { return distance_; }
  /// \}

  /// \name Configuration validation
  /// \{

  /// Set configuration validation methods
  /// Before starting tos solve a path planning problem, the initial and
  /// goal configurations are checked for validity.
  void configValidation(const ConfigValidationsPtr_t& configValidations) {
    configValidations_ = configValidations;
  }
  /// Get configuration validation methods
  const ConfigValidationsPtr_t& configValidations() const {
    return configValidations_;
  }

  /// Clear the ConfigValidations
  /// by calling ConfigValidations::clear
  void clearConfigValidations();

  /// Add a config validation method
  void addConfigValidation(const ConfigValidationPtr_t& configValidation);

  /// \name Path validation
  /// \{
  /// Set path validation method
  virtual void pathValidation(const PathValidationPtr_t& pathValidation);

  /// Get path validation method
  PathValidationPtr_t pathValidation() const { return pathValidation_; }
  /// \}

  /// \name Configuration shooter
  /// \{
  /// Set configuration shooter method
  void configurationShooter(
      const ConfigurationShooterPtr_t& configurationShooter);

  /// Get path validation method
  ConfigurationShooterPtr_t configurationShooter() const {
    return configurationShooter_;
  }
  /// \}

  /// \name Path projector
  /// \{
  /// Set path projector method
  void pathProjector(const PathProjectorPtr_t& pathProjector) {
    pathProjector_ = pathProjector;
  }

  /// Get path projector method
  PathProjectorPtr_t pathProjector() const { return pathProjector_; }
  /// \}

  /// \name Constraints applicable to the robot
  /// \{

  /// Set constraint set
  /// \param constraints a set of constraints
  /// If problem contains a steering method, constraints are passed to
  /// the steering method.
  void constraints(const ConstraintSetPtr_t& constraints) {
    constraints_ = constraints;
    if (steeringMethod_) steeringMethod_->constraints(constraints);
  }

  /// Get constraint set
  const ConstraintSetPtr_t& constraints() const { return constraints_; }
  /// \}

  /// Check that problem is well formulated
  virtual void checkProblem() const;

  /// \name Obstacles
  /// \{

  /// Add obstacle to the list.
  /// \param object a new object.
  void addObstacle(const CollisionObjectPtr_t& object);
  /// Remove a collision pair between a joint and an obstacle
  /// \param joint that holds the inner objects,
  /// \param obstacle to remove.
  void removeObstacleFromJoint(const JointPtr_t& joint,
                               const CollisionObjectConstPtr_t& obstacle);

  /// Build matrix of relative motions between joints
  ///
  /// Loop over constraints in the current constraint set
  /// (see Problem::constraints) and for each LockedJoint and each
  /// constraints::RelativeTransformation, fill a matrix the column and
  /// rows represent joints and the values are the following
  /// \li RelativeMotionType::Constrained when the two joints are rigidly
  ///     fixed by the constraint,
  /// \li RelativeMotionType::Parameterized when the two joints are rigidly
  ///     fixed by the constraint, the relative position is a parameter
  ///     constant along path, but that can change for different paths,
  /// \li RelativeMotionType::Unconstrained when the two joints are not
  ///     rigidly fixed by the constraint.
  ///
  /// \note the matrix is passed to the current configuration validation
  ///    instance (Problem::configValidation) and to the current
  /// path validation instance (Problem::pathValidation).
  void filterCollisionPairs();

  /// \copydoc ObstacleUserInterface::setSecurityMargins
  void setSecurityMargins(const matrix_t& securityMatrix);

  /// Vector of objects considered for collision detection
  const ObjectStdVector_t& collisionObstacles() const;
  /// Set the vector of objects considered for collision detection
  void collisionObstacles(const ObjectStdVector_t& collisionObstacles);
  /// \}

  /// Get a parameter named name.
  ///
  /// \param name of the parameter.
  const Parameter& getParameter(const std::string& name) const {
    if (parameters.has(name))
      return parameters.get(name);
    else
      return parameterDescription(name).defaultValue();
  }

  /// Set a parameter named name.
  ///
  /// \param name of the parameter.
  /// \param value value of the parameter
  /// \throw std::invalid_argument if a parameter exists but has a different
  ///        type.
  void setParameter(const std::string& name, const Parameter& value);

  /// Declare a parameter
  /// In shared library, use the following snippet in your cc file:
  /// \code{.cpp}
  /// HPP_START_PARAMETER_DECLARATION(name)
  /// Problem::declareParameter (ParameterDescription (
  ///       Parameter::FLOAT,
  ///       "name",
  ///       "doc",
  ///       Parameter(value)));
  /// HPP_END_PARAMETER_DECLARATION(name)
  /// \endcode
  static void declareParameter(const ParameterDescription& desc);

  /// Get all the parameter descriptions
  static const Container<ParameterDescription>& parameterDescriptions();

  /// Access one parameter description
  static const ParameterDescription& parameterDescription(
      const std::string& name);

  Container<Parameter> parameters;

 protected:
  /// \copydoc Problem::create(DevicePtr_t);
  Problem(DevicePtr_t robot);
  /// \copydoc Problem::create(DevicePtr_t);
  Problem(const Problem& other) = default;

  void init(ProblemWkPtr_t wkPtr);

 private:
  ProblemWkPtr_t wkPtr_;
  /// The robot
  DevicePtr_t robot_;
  /// Distance between configurations of the robot
  DistancePtr_t distance_;
  /// Shared pointer to initial configuration.
  Configuration_t initConf_;
  /// Shared pointer to problem target
  ProblemTargetPtr_t target_;
  /// Steering method associated to the problem
  SteeringMethodPtr_t steeringMethod_;
  /// Configuration validation
  ConfigValidationsPtr_t configValidations_;
  /// Path validation
  PathValidationPtr_t pathValidation_;
  /// Path projector
  PathProjectorPtr_t pathProjector_;
  /// List of obstacles
  ObjectStdVector_t collisionObstacles_;
  /// Set of constraints applicable to the robot
  ConstraintSetPtr_t constraints_;
  /// Configuration shooter
  ConfigurationShooterPtr_t configurationShooter_;
};  // class Problem
/// \}
}  // namespace core
}  // namespace hpp

#define HPP_START_PARAMETER_DECLARATION(name)        \
  struct HPP_CORE_DLLAPI __InitializerClass_##name { \
    __InitializerClass_##name() {
#define HPP_END_PARAMETER_DECLARATION(name)    \
  }                                            \
  }                                            \
  ;                                            \
  extern "C" {                                 \
  __InitializerClass_##name __instance_##name; \
  }

#endif  // HPP_CORE_PROBLEM_HH
