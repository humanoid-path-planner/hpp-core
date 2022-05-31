// Copyright (c) 2020, CNRS
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

#define HPP_DEBUG
#define BOOST_TEST_MODULE ReedsAndShepp
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>

using hpp::core::Parameter;
using hpp::core::PathPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::size_type;
using hpp::core::value_type;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::ConfigurationPtr_t;
using hpp::pinocchio::DevicePtr_t;

typedef hpp::core::distance::ReedsShepp Distance;
typedef hpp::core::distance::ReedsSheppPtr_t DistancePtr_t;
typedef hpp::core::steeringMethod::ReedsShepp SteeringMethod;
typedef hpp::core::steeringMethod::ReedsSheppPtr_t SteeringMethodPtr_t;

BOOST_AUTO_TEST_CASE(ReedsAndShepp) {
  DevicePtr_t robot =
      hpp::pinocchio::unittest::makeDevice(hpp::pinocchio::unittest::CarLike);
  BOOST_REQUIRE_EQUAL(robot->configSize(), 6);
  // Set bounds
  robot->rootJoint()->lowerBound(0, -10);
  robot->rootJoint()->lowerBound(1, -10);
  robot->rootJoint()->upperBound(0, 10);
  robot->rootJoint()->upperBound(1, 10);

  ProblemPtr_t problem(Problem::create(robot));
  // problem->setParameter("SteeringMethod/Carlike/turningRadius",
  // Parameter(2.));
  problem->setParameter(
      "SteeringMethod/Carlike/wheels",
      Parameter(std::string("wheel_frontright_joint,wheel_frontleft_joint")));
  // Rank of the wheels in the configuration vector
  SteeringMethodPtr_t sm(SteeringMethod::createWithGuess(problem));
  DistancePtr_t dist(Distance::create(problem));

  Configuration_t q1(robot->neutralConfiguration());
  Configuration_t q2(robot->neutralConfiguration());

  q1 << -2, 0, -1, 0, 0, 0;
  q1 << 2, 0, 1, 0, 0, 0;

  hppDout(info, "dist = " << (*dist)(q1, q2));
  PathPtr_t path((*sm)(q1, q2));
  hppDout(info, "path length = " << path->length());
}
