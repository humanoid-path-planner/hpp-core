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
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/core/discretized-collision-checking.hh>

namespace hpp {
  namespace core {

    extern std::string displayConfig (const Configuration_t& q);

    // ======================================================================

    Problem::Problem (DevicePtr_t robot) :
      robot_ (robot),
      distance_ (WeighedDistance::create (robot)),
      initConf_ (), goalConfigurations_ (),
      steeringMethod_ (new core::SteeringMethodStraight (robot)),
      pathValidation_ (DiscretizedCollisionChecking ::create (robot, 5e-2)),
      collisionObstacles_ (), distanceObstacles_ (), constraints_ ()
    {
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

    void Problem::addObstacle (const CollisionObjectShPtr_t& object,
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
      if (goalConfigurations_.size () == 0) {
	std::string msg ("No goal config in problem.");
	hppDout (error, msg);
	throw std::runtime_error (msg);
      }

      // Test that goal configurations are valid
      if (goalConfigurations_.empty ()) {
	std::string msg ("No Goal configuration");
	throw std::runtime_error (msg.c_str ());
      }
      for (ConfigConstIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	const ConfigurationPtr_t& goalConf (*it);
	if (validateConfig (robot (), *goalConf) != true) {
	  std::string msg ("One goal configuration not valid.");
	  hppDout (error, msg);
	  hppDout (error, displayConfig (*goalConf));
	  throw std::runtime_error (msg.c_str ());
	}
      }
    }

    // ======================================================================

    void Problem::validateInitConfig () const
    {
      if (!validateConfig (robot (), *initConf_)) {
	std::string msg ("Initial configuration is valid.");
	hppDout (info, msg);
	throw std::runtime_error (msg);
      }
    }

    bool Problem::validateConfig (const DevicePtr_t& device,
				  const Configuration_t& config) const
    {
      device->currentConfiguration (config);
      device->computeForwardKinematics ();
      return !(device->collisionTest ());
    }

  } // namespace core
} // namespace hpp
