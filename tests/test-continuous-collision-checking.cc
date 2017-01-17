// Copyright (c) 2017, LAAS-CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-constraints.
// hpp-constraints is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-constraints is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-constraints. If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE ContinuousCollisionChecking

#include <boost/test/included/unit_test.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/continuous-collision-checking/dichotomy.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method-straight.hh>

using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

using hpp::pinocchio::urdf::loadRobotModel;

using hpp::core::BasicConfigurationShooter;
using hpp::core::ConfigurationPtr_t;
using hpp::core::ConfigurationShooterPtr_t;
using hpp::core::continuousCollisionChecking::Dichotomy;
using hpp::core::continuousCollisionChecking::Progressive;
using hpp::core::DiscretizedCollisionChecking;
using hpp::core::PathPtr_t;
using hpp::core::PathValidationPtr_t;
using hpp::core::PathValidationReportPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::SteeringMethodStraight;

BOOST_AUTO_TEST_SUITE (test_hpp_core)

BOOST_AUTO_TEST_CASE (continuous_collision_checking)
{
  // Load robot model (ur5)
  DevicePtr_t robot (Device::create ("ur5"));
  loadRobotModel (robot, "anchor", "ur_description", "ur5_gripper", "", "");

  // Create configuration shooter
  ConfigurationShooterPtr_t shooter (BasicConfigurationShooter::create (robot));
  
  // create steering method
  ProblemPtr_t problem (new Problem (robot));
  SteeringMethodPtr_t sm (SteeringMethodStraight::create (problem));

  // create path validation objects
  PathValidationPtr_t dichotomy (Dichotomy::create (robot, 0));
  PathValidationPtr_t progressive (Progressive::create (robot, 0.05));
  PathValidationPtr_t discretized (DiscretizedCollisionChecking::create
				   (robot, 0.05));

  //  create random paths and test them with different validation instances
  for (std::size_t i=0; i<1000; ++i) {
    ConfigurationPtr_t q1 (shooter->shoot ());
    ConfigurationPtr_t q2 (shooter->shoot ());
    PathValidationReportPtr_t report1;
    PathValidationReportPtr_t report2;
    PathValidationReportPtr_t report3;
    PathPtr_t path ((*sm) (*q1, *q2));
    PathPtr_t validPart;
    bool res1 (discretized->validate (path, false, validPart, report1));
    bool res2 (progressive->validate  (path, false, validPart, report2));
    bool res3 (dichotomy->validate  (path, false, validPart, report3));

    if (!res1) {
      BOOST_CHECK (!res2);
      if (res2) {
	hppDout (error, "Progressive failed to detect collision for q1="
		 << q1->transpose () << ", q2=" << q2->transpose ());
	hppDout (error, *report1);
      }
      BOOST_CHECK (!res3);
      if (res3) {
	hppDout (error, "Dichotomy failed to detect collision for q1="
		 << q1->transpose () << ", q2=" << q2->transpose ());
	hppDout (error, *report1);
      }
    }
  }
  // delete problem
  delete problem;
}

BOOST_AUTO_TEST_SUITE_END()
