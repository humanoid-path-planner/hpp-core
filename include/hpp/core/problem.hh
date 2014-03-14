
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

# include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    /// Defines a path planning problem for one robot.
    /// A path planning problem is defined by
    /// \li a robot: instance of class hpp::model::Device,
    /// \li a set of obstacles: a list of hpp::model::CollisionObject,
    /// \li initial and goal configurations,
    /// \li a SteeringMethod to handle the robot dynamics,
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
      /// Get number of goal configuration.
      const Configurations_t& goalConfigs () const;
      /// Add goal configuration.
      void addGoalConfig (const ConfigurationPtr_t& config);
      /// Reset the set of goal configurations
      void resetGoalConfigs ();

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

      /// \name Path validation
      /// \{
      /// Set path validation method
      void pathValidation (const PathValidationPtr_t& pathValidation)
      {
	pathValidation_ = pathValidation;
      }

      /// Get path validation method
      PathValidationPtr_t pathValidation () const
      {
	return pathValidation_;
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
      void checkProblem () const;

      /// \name Obstacles
      /// \{

      /// Add obstacle to the list.
      /// \param object a new object.
      /// \param collision whether collision checking should be performed
      ///        for this object.
      /// \param distance whether distance computation should be performed
      ///        for this object.
      void addObstacle (const CollisionObjectShPtr_t& object, bool collision,
			bool distance);
      /// List of objects considered for collision detection
      const ObjectVector_t& collisionObstacles () const;
      /// List of objects considered for computation
      const ObjectVector_t& distanceObstacles () const;


      /// \}

    private :
      /// Validate configuration and track validation reports.
      bool validateConfig (const DevicePtr_t& device,
			   const Configuration_t& inConfig) const;

      /// Validate initial configuration
      void validateInitConfig () const;

      /// The robot
      DevicePtr_t robot_;
      /// Distance between configurations of the robot
      DistancePtr_t distance_;
      /// Shared pointer to initial configuration.
      ConfigurationPtr_t initConf_;
      /// Shared pointer to goal configuration.
      Configurations_t goalConfigurations_;
      /// Steering method associated to the problem
      SteeringMethodPtr_t steeringMethod_;
      /// Path validation
      PathValidationPtr_t pathValidation_;
      /// List of obstacles
      ObjectVector_t collisionObstacles_;
      /// List of obstacles
      ObjectVector_t distanceObstacles_;
      /// Set of constraints applicable to the robot
      ConstraintSetPtr_t constraints_;
    }; // class Problem
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_HH
