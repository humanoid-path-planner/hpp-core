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

#include <hpp/core/problem.hh>

#include <iostream>

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/continuous-collision-checking/dichotomy.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/basic-configuration-shooter.hh>

namespace hpp {
  namespace core {

    // ======================================================================

    Problem::Problem (DevicePtr_t robot) :
      robot_ (robot),
      distance_ (WeighedDistance::create (robot)),
      initConf_ (), goalConfigurations_ (), target_ (),
      steeringMethod_ (steeringMethod::Straight::create (*this)),
      configValidations_ (),
      pathValidation_ (DiscretizedCollisionChecking::create
		       (robot, 0.05)),
      collisionObstacles_ (), constraints_ (),
      configurationShooter_(BasicConfigurationShooter::create (robot))
    {
      resetConfigValidations();

      parameters.add("PathOptimizersNumberOfLoops", (std::size_t)5);
      parameters.add("PathProjectionHessianBound", (value_type)-1);
      parameters.add("PathProjectionMinimalDist", (value_type)1e-3);
      parameters.add("kPRMstar/numberOfNodes", (size_type)100);
    }

    // ======================================================================

    Problem::~Problem ()
    {
    }

    // ======================================================================

    void Problem::initConfig (const ConfigurationPtr_t& config)
    {
      initConf_ = config;
    }

    // ======================================================================

    const Configurations_t& Problem::goalConfigs () const
    {
      return goalConfigurations_;
    }

    // ======================================================================

    void Problem::addGoalConfig (const ConfigurationPtr_t& config)
    {
      goalConfigurations_.push_back (config);
    }

    // ======================================================================

    void Problem::resetGoalConfigs ()
    {
      goalConfigurations_.clear ();
    }

    // ======================================================================

    void Problem::resetConfigValidations ()
    {
      configValidations_ = ConfigValidations::create ();
    }

    // ======================================================================

    void Problem::clearConfigValidations ()
    {
      configValidations_->clear ();
    }

    // ======================================================================

    const ObjectStdVector_t& Problem::collisionObstacles () const
    {
      return collisionObstacles_;
    }

    // ======================================================================

    void Problem::collisionObstacles (const ObjectStdVector_t& collisionObstacles)
    {
        collisionObstacles_.clear ();
        // pass the local vector of collisions object to the problem
        for (ObjectStdVector_t::const_iterator itObj = collisionObstacles.begin();
             itObj != collisionObstacles.end(); ++itObj) {
            addObstacle (*itObj);
        }
    }

    // ======================================================================

    void Problem::addObstacle (const CollisionObjectPtr_t& object)
    {
      // Add object in local list
      collisionObstacles_.push_back (object);
      // Add obstacle to path validation method
      if (pathValidation_) {
	pathValidation_->addObstacle (object);
      }
      if (configValidations_) {
	configValidations_->addObstacle (object);
      }
    }

    // ======================================================================

    void Problem::removeObstacleFromJoint (const JointPtr_t& joint,
					   const CollisionObjectConstPtr_t& obstacle)
    {
      if (pathValidation_) {
	pathValidation_->removeObstacleFromJoint (joint, obstacle);
      }
      if (configValidations_) {
	configValidations_->removeObstacleFromJoint (joint, obstacle);
      }
    }
    // ======================================================================

    void Problem::filterCollisionPairs ()
    {
      RelativeMotion::matrix_type matrix = RelativeMotion::matrix (robot_);
      RelativeMotion::fromConstraint (matrix, robot_, constraints_);
      hppDout (info, "RelativeMotion matrix:\n" << matrix);

      if (pathValidation_) {
	pathValidation_->filterCollisionPairs (matrix);
      }
      if (configValidations_) {
	configValidations_->filterCollisionPairs (matrix);
      }
    }

    // ======================================================================

    void Problem::pathValidation (const PathValidationPtr_t& pathValidation)
    {
      pathValidation_ = pathValidation;
      // Insert obstacles in path validation object
      for (ObjectStdVector_t::const_iterator it =  collisionObstacles_.begin ();
	   it != collisionObstacles_.end (); ++it) {
	pathValidation_->addObstacle (*it);
      }
    }

    // ======================================================================

    void Problem::addConfigValidation (const ConfigValidationPtr_t& configValidation)
    {
      configValidations_->add ( configValidation );
    }

    // ======================================================================

    void Problem::configurationShooter (const ConfigurationShooterPtr_t& configurationShooter)
    {
      configurationShooter_ = configurationShooter;
    }

    // ======================================================================

    void Problem::checkProblem () const
    {
      if (!robot ()) {
	std::string msg ("No device in problem.");
	hppDout (error, msg);
	throw std::runtime_error (msg);
      }

      if (!initConfig ()) {
	std::string msg ("No init config in problem.");
	hppDout (error, msg);
	throw std::runtime_error (msg);
      }

      ValidationReportPtr_t report;
      if (!configValidations_->validate (*initConf_, report)) {
	std::ostringstream oss;
	oss << *report;
	throw std::runtime_error (oss.str ());
      }

      for (ConfigConstIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	const ConfigurationPtr_t& goalConf (*it);
	if (!configValidations_->validate (*goalConf, report)) {
	  std::ostringstream oss;
	  oss << *report;
	  throw std::runtime_error (oss.str ());
	}
      }
    }

    // ======================================================================

    void Problem::setParameter (const std::string& name, const boost::any& value)
      throw (std::invalid_argument)
    {
      if (parameters.has(name)) {
        const boost::any& val = parameters.get(name);
        if (value.type() != val.type()) {
          std::string ret = "Wrong boost::any type. Expects ";
          ret += val.type().name();
          throw std::invalid_argument (ret.c_str());
        }
      }
      parameters.add (name, value);
    }

    // ======================================================================

  } // namespace core
} // namespace hpp
