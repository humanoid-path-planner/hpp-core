//
// Copyright (c) 2014 CNRS
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

#define BOOST_TEST_MODULE gradient_based

#include <boost/test/included/unit_test.hpp>
#include <cmath>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/fwd.hpp>

using namespace hpp::core;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_SUITE(test_hpp_core)

// Build a box robot moving in the plane
DevicePtr_t createRobot() {
  std::string urdf(
      "<robot name='test'>"
      "<link name='body'>"
      "<collision>"
      "<geometry>"
      "<box size='1 2 1'/>"
      "</geometry>"
      "</collision>"
      "</link>"
      "</robot>");

  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "", "planar", urdf, "");

  BOOST_REQUIRE_EQUAL(robot->configSize(), 4);
  BOOST_REQUIRE_EQUAL(robot->numberDof(), 3);

  JointPtr_t rj = robot->getJointByName("root_joint");
  BOOST_REQUIRE(rj);
  rj->upperBound(0, 2);
  rj->upperBound(1, 2);
  rj->lowerBound(0, -2);
  rj->lowerBound(1, -2);
  return robot;
}

// Build a box robot moving in the plane
//
// Create a circular path from (-1,0) to (1,0) with nominal orientation
// with 3 waypoints of various orientations.
// Optimal path should be a straight line: all waypoints aligned.
// Check waypoints with expected value

BOOST_AUTO_TEST_CASE(BFGS) {
  DevicePtr_t robot = createRobot();
  Configuration_t q0(robot->configSize());
  Configuration_t q1(robot->configSize());
  Configuration_t q2(robot->configSize());
  Configuration_t q3(robot->configSize());
  Configuration_t q4(robot->configSize());
  value_type s = sqrt(2) / 2;
  q0(0) = -1;
  q0(1) = 0;
  q0(2) = 1;
  q0(3) = 0;
  q1(0) = -s;
  q1(1) = s;
  q1(2) = s;
  q1(3) = s;
  q2(0) = 0;
  q2(1) = 1;
  q2(2) = 0;
  q2(3) = 1;
  q3(0) = s;
  q3(1) = s;
  q3(2) = s;
  q3(3) = s;
  q4(0) = 1;
  q4(1) = 0;
  q4(2) = 1;
  q4(3) = 0;

  ProblemPtr_t problem = Problem::create(robot);
  SteeringMethodPtr_t sm = problem->steeringMethod();
  PathVectorPtr_t path =
      PathVector::create(robot->configSize(), robot->numberDof());
  path->appendPath((*sm)(q0, q1));
  path->appendPath((*sm)(q1, q2));
  path->appendPath((*sm)(q2, q3));
  path->appendPath((*sm)(q3, q4));
  problem->setParameter("SplineGradientBased/alphaInit",
                        hpp::core::Parameter(1.));
  problem->setParameter("SplineGradientBased/costThreshold",
                        hpp::core::Parameter(1e-6));
  PathOptimizerPtr_t pathOptimizer(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 1>::create(
          problem));
  PathVectorPtr_t optimizedPath(pathOptimizer->optimize(path));
  Configuration_t p0(robot->configSize());
  Configuration_t p1(robot->configSize());
  Configuration_t p2(robot->configSize());
  Configuration_t p3(robot->configSize());
  Configuration_t p4(robot->configSize());
  BOOST_CHECK(optimizedPath->numberPaths() == 4);
  p0 = optimizedPath->initial();
  p1 = optimizedPath->pathAtRank(0)->end();
  p2 = optimizedPath->pathAtRank(1)->end();
  p3 = optimizedPath->pathAtRank(2)->end();
  p4 = optimizedPath->pathAtRank(3)->end();
  Configuration_t r0(robot->configSize());
  Configuration_t r1(robot->configSize());
  Configuration_t r2(robot->configSize());
  Configuration_t r3(robot->configSize());
  Configuration_t r4(robot->configSize());
  r0 << -1, 0, 1, 0;
  r1 << -0.5, 0, 1, 0;
  r2 << 0.0, 0, 1, 0;
  r3 << 0.5, 0, 1, 0;
  r4 << 1, 0, 1, 0;

  BOOST_CHECK((p0 - r0).norm() < 1e-10);
  BOOST_CHECK((p1 - r1).norm() < 1e-10);
  BOOST_CHECK((p2 - r2).norm() < 1e-10);
  BOOST_CHECK((p3 - r3).norm() < 1e-10);
  BOOST_CHECK((p4 - r4).norm() < 1e-10);

  hppDout(info, (p0 - r0).norm());
  hppDout(info, (p1 - r1).norm());
  hppDout(info, (p2 - r2).norm());
  hppDout(info, (p3 - r3).norm());
  hppDout(info, (p4 - r4).norm());
}
BOOST_AUTO_TEST_SUITE_END()
