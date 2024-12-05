// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE problem
#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>

#include <boost/test/included/unit_test.hpp>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/fwd.hpp>

using namespace hpp::core;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE(memory_deallocation) {
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->robot(unittest::makeDevice(unittest::ManipulatorArm2));

  ProblemPtr_t problem = ps->problem();
  DeviceWkPtr_t dev = ps->robot();
  SteeringMethodWkPtr_t sm = problem->steeringMethod();
  DistanceWkPtr_t distance = problem->distance();
  RoadmapWkPtr_t roadmap = ps->roadmap();
  problem.reset();

  delete ps;

  BOOST_CHECK_MESSAGE(!dev.lock(), "Device was not deleted");
  BOOST_CHECK_MESSAGE(!sm.lock(), "Steering method was not deleted");
  BOOST_CHECK_MESSAGE(!distance.lock(), "Distance was not deleted");
  BOOST_CHECK_MESSAGE(!roadmap.lock(), "Roadmap was not deleted");
}

void pointMassProblem(const char* steeringMethod, const char* distance,
                      const char* pathValidation, value_type tolerance) {
  const char* urdfString =
      "<robot name='foo'><link name='base_link'>"
      "<collision><geometry><sphere radius='0.01'/></geometry></collision>"
      "</link></robot>";

  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->maxIterPathPlanning(50);

  DevicePtr_t robot = Device::create("point");
  urdf::loadModelFromString(robot, 0, "", "translation3d", urdfString, "");
  BOOST_REQUIRE_EQUAL(robot->configSize(), 3);
  robot->rootJoint()->lowerBound(0, -10);
  robot->rootJoint()->lowerBound(1, -10);
  robot->rootJoint()->lowerBound(2, -10);
  robot->rootJoint()->upperBound(0, 10);
  robot->rootJoint()->upperBound(1, 10);
  robot->rootJoint()->upperBound(2, 10);

  ps->robot(robot);
  ps->pathValidationType(pathValidation, tolerance);
  ps->steeringMethodType(steeringMethod);
  ps->distanceType(distance);

  CollisionGeometryPtr_t boxGeom(new coal::Box(0.3, 0.3, 0.3));
  SE3 boxTf(matrix3_t::Identity(), vector3_t(-2, 0, 0));
  ps->addObstacle("box", boxGeom, boxTf, true, true);

  Configuration_t qinit(robot->neutralConfiguration());
  Configuration_t qgoal(robot->neutralConfiguration());
  qgoal << -4, 0, 0;

  ps->initConfig(qinit);
  ps->addGoalConfig(qgoal);

  ps->solve();

  BOOST_CHECK(ps->roadmap()->nodes().size() > 2);
  BOOST_TEST_MESSAGE("Solved the problem with " << ps->roadmap()->nodes().size()
                                                << " nodes.");
}

void carLikeProblem(const char* steeringMethod, const char* distance,
                    const char* pathValidation, value_type tolerance) {
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->maxIterPathPlanning(50);

  DevicePtr_t robot = unittest::makeDevice(unittest::CarLike);
  BOOST_REQUIRE_EQUAL(robot->configSize(), 6);
  robot->rootJoint()->lowerBound(0, -10);
  robot->rootJoint()->lowerBound(1, -10);
  robot->rootJoint()->upperBound(0, 10);
  robot->rootJoint()->upperBound(1, 10);

  ps->robot(robot);
  ps->pathValidationType(pathValidation, tolerance);
  ps->steeringMethodType(steeringMethod);
  ps->distanceType(distance);

  CollisionGeometryPtr_t boxGeom(new coal::Box(0.3, 0.3, 0.3));
  SE3 boxTf(matrix3_t::Identity(), vector3_t(-2, 0, 0));
  ps->addObstacle("box", boxGeom, boxTf, true, true);

  Configuration_t qinit(robot->neutralConfiguration());
  Configuration_t qgoal(robot->neutralConfiguration());
  qgoal << -4, 0, 0, 1, 0, 0;

  ps->initConfig(qinit);
  ps->addGoalConfig(qgoal);

  ps->solve();

  BOOST_CHECK(ps->roadmap()->nodes().size() > 2);
  BOOST_TEST_MESSAGE("Solved the problem with " << ps->roadmap()->nodes().size()
                                                << " nodes.");
}

BOOST_AUTO_TEST_CASE(pointMass) {
  pointMassProblem("Straight", "Weighed", "Discretized", 0.05);
  pointMassProblem("Straight", "Weighed", "Progressive", 0.05);
  pointMassProblem("Straight", "Weighed", "Dichotomy", 0);
}

BOOST_AUTO_TEST_CASE(defaultProblem) {
  const char* urdfString =
      "<robot name='foo'><link name='base_link'>"
      "<collision><geometry><sphere radius='0.01'/></geometry></collision>"
      "</link></robot>";

  DevicePtr_t robot = Device::create("point");
  urdf::loadModelFromString(robot, 0, "", "translation3d", urdfString, "");
  BOOST_REQUIRE_EQUAL(robot->configSize(), 3);
  robot->rootJoint()->lowerBound(0, -10);
  robot->rootJoint()->lowerBound(1, -10);
  robot->rootJoint()->lowerBound(2, -10);
  robot->rootJoint()->upperBound(0, 10);
  robot->rootJoint()->upperBound(1, 10);
  robot->rootJoint()->upperBound(2, 10);

  ProblemPtr_t problem = Problem::create(robot);
  BOOST_REQUIRE(problem->constraints());

  Configuration_t qinit(robot->neutralConfiguration());
  Configuration_t qgoal(robot->neutralConfiguration());
  qgoal << -4, 0, 0;

  problem->initConfig(qinit);
  problem->addGoalConfig(qgoal);

  BOOST_CHECK_NO_THROW(problem->checkProblem());
}

BOOST_AUTO_TEST_CASE(carlike) {
  carLikeProblem("Straight", "Weighed", "Discretized", 0.05);
  carLikeProblem("Straight", "Weighed", "Progressive", 0.05);
  carLikeProblem("Straight", "Weighed", "Dichotomy", 0);
  carLikeProblem("ReedsShepp", "ReedsShepp", "Discretized", 0.05);
  // Not implemented
  // carLikeProblem ("ReedsShepp", "ReedsShepp", "Progressive", 0.05);
  // Not implemented
  // carLikeProblem ("ReedsShepp", "ReedsShepp", "Dichotomy"  , 0   );
}
