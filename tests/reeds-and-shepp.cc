// Copyright (c) 2020, CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#define HPP_DEBUG
#define BOOST_TEST_MODULE ReedsAndShepp
#include <boost/test/included/unit_test.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>

#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>

using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::ConfigurationPtr_t;
using hpp::core::PathPtr_t;
using hpp::core::Parameter;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::size_type;
using hpp::core::value_type;

typedef hpp::core::distance::ReedsShepp Distance;
typedef hpp::core::distance::ReedsSheppPtr_t DistancePtr_t;
typedef hpp::core::steeringMethod::ReedsShepp SteeringMethod;
typedef hpp::core::steeringMethod::ReedsSheppPtr_t SteeringMethodPtr_t;

BOOST_AUTO_TEST_CASE (ReedsAndShepp)
{
  DevicePtr_t robot = hpp::pinocchio::unittest::makeDevice
    (hpp::pinocchio::unittest::CarLike);
  BOOST_REQUIRE_EQUAL (robot->configSize(), 6);
  // Set bounds
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);

  ProblemPtr_t problem(Problem::create(robot));
  //problem->setParameter("SteeringMethod/Carlike/turningRadius", Parameter(2.));
  problem->setParameter("SteeringMethod/Carlike/wheels",
    Parameter(std::string("wheel_frontright_joint,wheel_frontleft_joint")));
  // Rank of the wheels in the configuration vector
  SteeringMethodPtr_t sm(SteeringMethod::createWithGuess(problem));
  DistancePtr_t dist(Distance::create(problem));

  Configuration_t q1(robot->neutralConfiguration());
  Configuration_t q2(robot->neutralConfiguration());

  q1 << -2, 0, -1, 0, 0, 0;
  q1 <<  2, 0,  1, 0, 0, 0;

  hppDout(info, "dist = " << (*dist)(q1, q2));
  PathPtr_t path((*sm)(q1,q2));
  hppDout(info, "path length = " << path->length());
}
