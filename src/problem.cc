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

#include <iostream>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/core/discretized-collision-checking.hh>

namespace hpp {
  namespace core {

    // ======================================================================

    Problem::Problem (DevicePtr_t robot) :
      robot_ (robot),
      distance_ (WeighedDistance::create (robot)),
      initConf_ (), goalConfigurations_ (),
      steeringMethod_ (new core::SteeringMethodStraight (robot)),
      configValidations_ (ConfigValidations::create ()),
      pathValidation_ (DiscretizedCollisionChecking ::create (robot, 5e-2)),
      collisionObstacles_ (), distanceObstacles_ (), constraints_ ()
    {
      configValidations_->add (CollisionValidation::create (robot));
      configValidations_->add (JointBoundValidation::create (robot));
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

    const ObjectVector_t& Problem::collisionObstacles () const
    {
      return collisionObstacles_;
    }

    // ======================================================================

    const ObjectVector_t& Problem::distanceObstacles () const
    {
      return distanceObstacles_;
    }

    // ======================================================================

    void Problem::collisionObstacles (const ObjectVector_t& collisionObstacles)
    {
      // reset outerCollisionObjects from the Device :
      for (ObjectVector_t::iterator itObj = collisionObstacles_.begin ();
	   itObj != collisionObstacles_.end (); ++itObj) {
	robot_->removeOuterObject (*itObj, true, false);
      }
      collisionObstacles_.clear ();
      // pass the local vector of collisions object to the problem
      for (ObjectVector_t::const_iterator itObj = collisionObstacles.begin();
	   itObj != collisionObstacles.end(); ++itObj) {
	addObstacle (*itObj, true, false);
      }
    }

    // ======================================================================

    /// Set the vector of objects considered for distance computation
    void Problem::distanceObstacles (const ObjectVector_t& distanceObstacles)
    {
      // reset outerDistanceObjects from the Device :
      for (ObjectVector_t::iterator itObj = distanceObstacles_.begin ();
	   itObj != distanceObstacles_.end (); ++itObj) {
	robot_->removeOuterObject (*itObj, false, true);
      }
      distanceObstacles_.clear ();
      // pass the local vector of distances object to the problem
      for (ObjectVector_t::const_iterator itObj = distanceObstacles.begin();
	   itObj != distanceObstacles.end(); ++itObj) {
	addObstacle (*itObj, false, true);
      }
    }

    // ======================================================================

    void Problem::addObstacle (const CollisionObjectPtr_t& object,
			       bool collision, bool distance)
    {
      // Add object in local list
      if (collision)
	collisionObstacles_.push_back (object);
      if (distance)
	distanceObstacles_.push_back (object);
      // Add obstacle to robot
      robot_->addOuterObject (object, collision, distance);
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

      configValidations_->validate (*initConf_, true);

      if (goalConfigurations_.size () == 0) {
	std::string msg ("No goal config in problem.");
	hppDout (error, msg);
	throw std::runtime_error (msg);
      }

      for (ConfigConstIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	const ConfigurationPtr_t& goalConf (*it);
	configValidations_->validate (*goalConf, true);
      }
    }

    // ======================================================================

  } // namespace core
} // namespace hpp
