//
// Copyright (c) 2014 CNRS
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

#define BOOST_TEST_MODULE gradient_based

#include <cmath>
#include <boost/test/included/unit_test.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>

using namespace hpp::core;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

// Build a box robot moving in the plane
DevicePtr_t createRobot ()
{
  std::string urdf ("<robot name='test'>"
      "<link name='body'>"
        "<collision>"
          "<geometry>"
            "<box size='1 2 1'/>"
          "</geometry>"
        "</collision>"
      "</link>"
      "</robot>"
      );

  DevicePtr_t robot = Device::create ("test");
  urdf::loadModelFromString (robot, 0, "", "planar", urdf, "");

  BOOST_REQUIRE_EQUAL(robot->configSize(), 4);
  BOOST_REQUIRE_EQUAL(robot->numberDof (), 3);

  JointPtr_t rj = robot->getJointByName("root_joint");
  BOOST_REQUIRE(rj);
  rj->upperBound (0, 2);
  rj->upperBound (1, 2);
  rj->lowerBound (0,-2);
  rj->lowerBound (1,-2);
  return robot;
}

// Build a box robot moving in the plane
//
// Create a circular path from (-1,0) to (1,0) with nominal orientation
// with 3 waypoints of various orientations.
// Optimal path should be a straight line: all waypoints aligned.
// Check waypoints with expected value

BOOST_AUTO_TEST_CASE (BFGS)
{
  DevicePtr_t robot = createRobot ();
  Configuration_t q0 (robot->configSize ());
  Configuration_t q1 (robot->configSize ());
  Configuration_t q2 (robot->configSize ());
  Configuration_t q3 (robot->configSize ());
  Configuration_t q4 (robot->configSize ());
  value_type s = sqrt (2)/2;
  q0 (0) = -1; q0 (1) = 0; q0 (2) = 1; q0 (3) = 0;
  q1 (0) = -s; q1 (1) = s; q1 (2) = s; q1 (3) = s;
  q2 (0) = 0; q2 (1) = 1; q2 (2) = 0; q2 (3) = 1;
  q3 (0) = s; q3 (1) = s; q3 (2) = s; q3 (3) = s;
  q4 (0) = 1; q4 (1) = 0; q4 (2) = 1; q4 (3) = 0;

  ProblemPtr_t problem = Problem::create(robot);
  SteeringMethodPtr_t sm = problem->steeringMethod ();
  PathVectorPtr_t path = PathVector::create (robot->configSize (),
					     robot->numberDof ());
  path->appendPath ((*sm) (q0, q1));
  path->appendPath ((*sm) (q1, q2));
  path->appendPath ((*sm) (q2, q3));
  path->appendPath ((*sm) (q3, q4));
  problem->setParameter ("SplineGradientBased/alphaInit",
                        hpp::core::Parameter (1.));
  problem->setParameter ("SplineGradientBased/costThreshold",
                        hpp::core::Parameter (1e-6));
  PathOptimizerPtr_t pathOptimizer
    (pathOptimization::SplineGradientBased<path::BernsteinBasis, 1>::create
     (*problem));
  PathVectorPtr_t optimizedPath (pathOptimizer->optimize (path));
  Configuration_t p0 (robot->configSize ());
  Configuration_t p1 (robot->configSize ());
  Configuration_t p2 (robot->configSize ());
  Configuration_t p3 (robot->configSize ());
  Configuration_t p4 (robot->configSize ());
  BOOST_CHECK (optimizedPath->numberPaths () == 4);
  p0 = optimizedPath->initial ();
  p1 = optimizedPath->pathAtRank (0)->end ();
  p2 = optimizedPath->pathAtRank (1)->end ();
  p3 = optimizedPath->pathAtRank (2)->end ();
  p4 = optimizedPath->pathAtRank (3)->end ();
  Configuration_t r0 (robot->configSize ());
  Configuration_t r1 (robot->configSize ());
  Configuration_t r2 (robot->configSize ());
  Configuration_t r3 (robot->configSize ());
  Configuration_t r4 (robot->configSize ());
  r0 << -1,  0,1,0;
  r1 << -0.5,0,1,0;
  r2 <<  0.0,0,1,0;
  r3 <<  0.5,0,1,0;
  r4 <<  1,  0,1,0;

  BOOST_CHECK ((p0 - r0).norm () < 1e-10);
  BOOST_CHECK ((p1 - r1).norm () < 1e-10);
  BOOST_CHECK ((p2 - r2).norm () < 1e-10);
  BOOST_CHECK ((p3 - r3).norm () < 1e-10);
  BOOST_CHECK ((p4 - r4).norm () < 1e-10);

  hppDout (info, (p0 - r0).norm ());
  hppDout (info, (p1 - r1).norm ());
  hppDout (info, (p2 - r2).norm ());
  hppDout (info, (p3 - r3).norm ());
  hppDout (info, (p4 - r4).norm ());
}
BOOST_AUTO_TEST_SUITE_END()
