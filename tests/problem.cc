// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE problem
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/roadmap.hh>

using namespace hpp::core;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE (memory_deallocation)
{
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->robot(unittest::makeDevice(unittest::ManipulatorArm2));

  ProblemPtr_t problem = ps->problem();
  DeviceWkPtr_t dev = ps->robot();
  SteeringMethodWkPtr_t sm = problem->steeringMethod();
  DistanceWkPtr_t distance = problem->distance ();
  RoadmapWkPtr_t roadmap = ps->roadmap ();
  problem.reset();

  delete ps;

  BOOST_CHECK_MESSAGE( !dev     .lock(), "Device was not deleted");
  BOOST_CHECK_MESSAGE( !sm      .lock(), "Steering method was not deleted");
  BOOST_CHECK_MESSAGE( !distance.lock(), "Distance was not deleted");
  BOOST_CHECK_MESSAGE( !roadmap .lock(), "Roadmap was not deleted");
}

void pointMassProblem (const char* steeringMethod,
    const char* distance,
    const char* pathValidation, value_type tolerance)
{
  const char* urdfString =
      "<robot name='foo'><link name='base_link'>"
      "<collision><geometry><sphere radius='0.01'/></geometry></collision>"
      "</link></robot>";

  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->maxIterPathPlanning (50);

  DevicePtr_t robot = Device::create ("point");
  urdf::loadModelFromString (robot, 0, "", "translation3d", urdfString, "");
  BOOST_REQUIRE_EQUAL (robot->configSize(), 3);
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->lowerBound (2, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);
  robot->rootJoint()->upperBound (2,  10);

  ps->robot(robot);
  ps->pathValidationType (pathValidation, tolerance);
  ps->steeringMethodType (steeringMethod);
  ps->distanceType (distance);

  FclCollisionObject box (
      hpp::fcl::CollisionGeometryPtr_t (new hpp::fcl::Box (0.3, 0.3, 0.3)),
      matrix3_t::Identity(), vector3_t (-2, 0, 0));
  ps->addObstacle ("box", box, true, true);

  ConfigurationPtr_t qinit (new Configuration_t (robot->neutralConfiguration()));
  ConfigurationPtr_t qgoal (new Configuration_t (robot->neutralConfiguration()));
  *qgoal << -4, 0, 0;

  ps->initConfig (qinit);
  ps->addGoalConfig (qgoal);

  ps->solve ();

  BOOST_CHECK (ps->roadmap()->nodes().size() > 2);
  BOOST_MESSAGE ("Solved the problem with " << ps->roadmap()->nodes().size()
      << " nodes.");
}

void carLikeProblem (const char* steeringMethod,
    const char* distance,
    const char* pathValidation, value_type tolerance)
{
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->maxIterPathPlanning (50);

  DevicePtr_t robot = unittest::makeDevice(unittest::CarLike);
  BOOST_REQUIRE_EQUAL (robot->configSize(), 6);
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);

  ps->robot(robot);
  ps->pathValidationType (pathValidation, tolerance);
  ps->steeringMethodType (steeringMethod);
  ps->distanceType (distance);

  FclCollisionObject box (
      hpp::fcl::CollisionGeometryPtr_t (new hpp::fcl::Box (0.3, 0.3, 0.3)),
      matrix3_t::Identity(), vector3_t (-2, 0, 0));
  ps->addObstacle ("box", box, true, true);

  ConfigurationPtr_t qinit (new Configuration_t (robot->neutralConfiguration()));
  ConfigurationPtr_t qgoal (new Configuration_t (robot->neutralConfiguration()));
  *qgoal << -4, 0, 0, 1, 0, 0;

  ps->initConfig (qinit);
  ps->addGoalConfig (qgoal);

  ps->solve ();

  BOOST_CHECK (ps->roadmap()->nodes().size() > 2);
  BOOST_MESSAGE ("Solved the problem with " << ps->roadmap()->nodes().size()
      << " nodes.");
}

BOOST_AUTO_TEST_CASE (pointMass)
{
  pointMassProblem ("Straight", "Weighed", "Discretized", 0.05);
  pointMassProblem ("Straight", "Weighed", "Progressive", 0.05);
  pointMassProblem ("Straight", "Weighed", "Dichotomy"  , 0   );
}

BOOST_AUTO_TEST_CASE (carlike)
{
  carLikeProblem ("Straight", "Weighed", "Discretized", 0.05);
  carLikeProblem ("Straight", "Weighed", "Progressive", 0.05);
  carLikeProblem ("Straight", "Weighed", "Dichotomy"  , 0   );
  carLikeProblem ("ReedsShepp", "ReedsShepp", "Discretized", 0.05);
  // Not implemented
  // carLikeProblem ("ReedsShepp", "ReedsShepp", "Progressive", 0.05);
  // Not implemented
  // carLikeProblem ("ReedsShepp", "ReedsShepp", "Dichotomy"  , 0   );
}
