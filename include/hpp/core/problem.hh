
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

#ifndef HPP_CORE_PROBLEM_HH
# define HPP_CORE_PROBLEM_HH

# include <hpp/model/device.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/config.hh>
# include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Defines a path planning problem for one robot.
    /// A path planning problem is defined by
    /// \li a robot: instance of class hpp::model::Device,
    /// \li a set of obstacles: a list of hpp::model::CollisionObject,
    /// \li initial and goal configurations,
    /// \li a SteeringMethod to handle the robot dynamics,
    /// Additional objects are stored in this object:
    /// \li a method to validate paths,
    /// \li a set of methods to validate configurations. Default methods are
    /// collision checking and joint bound checking.
    class HPP_CORE_DLLAPI Problem
    {
    public:
      /// Create a path planning problem.
      /// \param robot robot associated to the path planning problem.
      Problem (DevicePtr_t robot);

      virtual ~Problem ();

      /// \name Problem definition
      /// \{

      /// return shared pointer to robot.
      const DevicePtr_t& robot () const {
	return robot_;
      }

      /// set robot
      /// Set steering method, distance function and path validation to
      /// default values
      void robot (const DevicePtr_t& device);

      /// Get shared pointer to initial configuration.
      const ConfigurationPtr_t& initConfig () const
      {
	return initConf_;
      }
      /// Set initial configuration.
      void initConfig (const ConfigurationPtr_t& inConfig);
      /// Set the target
      void target (const ProblemTargetPtr_t& target)
      {
        target_ = target;
      }
      /// Get the target
      const ProblemTargetPtr_t& target () const
      {
        return target_;
      }
      /// Get number of goal configuration.
      /// \deprecated use target ()
      const Configurations_t& goalConfigs () const HPP_CORE_DEPRECATED;
      /// Add goal configuration.
      /// \deprecated use target ()
      void addGoalConfig (const ConfigurationPtr_t& config) HPP_CORE_DEPRECATED;
      /// Reset the set of goal configurations
      /// \deprecated use target ()
      void resetGoalConfigs () HPP_CORE_DEPRECATED;

      /// \}

      /// \name Steering method and distance function
      /// \{

      /// Set steering method
      /// \param sm steering method.
      /// If problem contains constraints they are passed to the steering
      /// method.
      void steeringMethod (const SteeringMethodPtr_t& sm) {
	steeringMethod_ = sm;
	if (constraints_) steeringMethod_->constraints (constraints_);
      }

      /// Get steering method
      SteeringMethodPtr_t steeringMethod () const {
	return steeringMethod_;
      }

      /// Set distance between configurations
      void distance (const DistancePtr_t& distance)
      {
	distance_ = distance;
      }
      /// Get distance between configuration
      const DistancePtr_t& distance () const
      {
	return distance_;
      }
      /// \}

      /// \name Configuration validation
      /// \{

      /// Set configuration validation methods
      /// Before starting tos solve a path planning problem, the initial and
      /// goal configurations are checked for validity.
      void configValidation (const ConfigValidationsPtr_t& configValidations)
      {
	configValidations_ = configValidations;
      }
      /// Get configuration validation methods
      const ConfigValidationsPtr_t& configValidations () const
      {
	return configValidations_;
      }

      /// \name Path validation
      /// \{
      /// Set path validation method
      void pathValidation (const PathValidationPtr_t& pathValidation);

      /// Get path validation method
      PathValidationPtr_t pathValidation () const
      {
	return pathValidation_;
      }
      /// \}


      /// \name Configuration shooter
      /// \{
      /// Set configuration shooter method
      void configurationShooter (const ConfigurationShooterPtr_t& configurationShooter);

      /// Get path validation method
      ConfigurationShooterPtr_t configurationShooter () const
      {
	return configurationShooter_;
      }
      /// \}

      /// \name Path projector
      /// \{
      /// Set path projector method
      void pathProjector (const PathProjectorPtr_t& pathProjector)
      {
        pathProjector_ = pathProjector;
      }

      /// Get path projector method
      PathProjectorPtr_t pathProjector () const
      {
	return pathProjector_;
      }
      /// \}

      /// \name Constraints applicable to the robot
      /// \{

      /// Set constraint set
      /// \param constraints a set of constraints
      /// If problem contains a steering method, constraints are passed to
      /// the steering method.
      void constraints (const ConstraintSetPtr_t& constraints)
      {
	constraints_ = constraints;
	if (steeringMethod_) steeringMethod_->constraints (constraints);
      }

      ///Get constraint set
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }
      /// \}

      /// Check that problem is well formulated
      virtual void checkProblem () const;

      /// \name Obstacles
      /// \{

      /// Add obstacle to the list.
      /// \param object a new object.
      void addObstacle (const CollisionObjectPtr_t& object);
      /// Remove a collision pair between a joint and an obstacle
      /// \param the joint that holds the inner objects,
      /// \param the obstacle to remove.
      void removeObstacleFromJoint (const JointPtr_t& joint,
				    const CollisionObjectPtr_t& obstacle);

      /// Vector of objects considered for collision detection
      const ObjectVector_t& collisionObstacles () const;
      /// Set the vector of objects considered for collision detection
      void collisionObstacles (const ObjectVector_t& collisionObstacles);
      /// \}

    private :
      /// The robot
      DevicePtr_t robot_;
      /// Distance between configurations of the robot
      DistancePtr_t distance_;
      /// Shared pointer to initial configuration.
      ConfigurationPtr_t initConf_;
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
      ObjectVector_t collisionObstacles_;
      /// Set of constraints applicable to the robot
      ConstraintSetPtr_t constraints_;
      /// Configuration shooter
      ConfigurationShooterPtr_t configurationShooter_;
    }; // class Problem
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_HH
