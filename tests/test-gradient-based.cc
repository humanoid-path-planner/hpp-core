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
#include <hpp/fcl/shape/geometric_shapes.h>

#include <boost/test/included/unit_test.hpp>
#include <cmath>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>

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
      "<cylinder length='.2' radius='.1'/>"
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

// Build a cylindrical robot moving in the plane
//
// Create a circular path from (-1,0) to (1,0) with nominal orientation
// with 3 waypoints of various orientations.
// Optimal path should be a straight line: all waypoints aligned.
// Check waypoints with expected value

BOOST_AUTO_TEST_CASE(spline_optimization) {
  Eigen::IOFormat eigenFmt(Eigen::FullPrecision);
  ::hpp::debug::setVerbosityLevel(100);
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
  problem->setParameter("SplineGradientBased/QPAccuracy",
                        hpp::core::Parameter(1e-10));
  PathOptimizerPtr_t pathOptimizer1(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 1>::create(
          problem));
  PathOptimizerPtr_t pathOptimizer3(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 3>::create(
          problem));
  PathOptimizerPtr_t pathOptimizer5(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 5>::create(
          problem));
  PathVectorPtr_t optimizedPath1(pathOptimizer1->optimize(path));
  PathVectorPtr_t optimizedPath3(pathOptimizer3->optimize(path));
  PathVectorPtr_t optimizedPath5(pathOptimizer5->optimize(path));
  BOOST_CHECK(optimizedPath1->numberPaths() == 4);
  BOOST_CHECK(optimizedPath3->numberPaths() == 4);
  BOOST_CHECK(optimizedPath5->numberPaths() == 4);
  Configuration_t p00 = optimizedPath1->pathAtRank(0)->initial();
  Configuration_t p01 = optimizedPath1->pathAtRank(0)->end();
  Configuration_t p10 = optimizedPath1->pathAtRank(1)->initial();
  Configuration_t p11 = optimizedPath1->pathAtRank(1)->end();
  Configuration_t p20 = optimizedPath1->pathAtRank(2)->initial();
  Configuration_t p21 = optimizedPath1->pathAtRank(2)->end();
  Configuration_t p30 = optimizedPath1->pathAtRank(3)->initial();
  Configuration_t p31 = optimizedPath1->pathAtRank(3)->end();
  Configuration_t r1(robot->configSize());
  Configuration_t r2(robot->configSize());
  Configuration_t r3(robot->configSize());
  // Test spline of degree 1
  r1 << -0.5, 0, 1, 0;
  r2 << 0.0, 0, 1, 0;
  r3 << 0.5, 0, 1, 0;

  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - r1).norm() < 1e-10);
  BOOST_CHECK((p10 - r1).norm() < 1e-10);
  BOOST_CHECK((p11 - r2).norm() < 1e-10);
  BOOST_CHECK((p20 - r2).norm() < 1e-10);
  BOOST_CHECK((p21 - r3).norm() < 1e-10);
  BOOST_CHECK((p30 - r3).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);

  // Test spline of degree 3
  p00 = optimizedPath3->pathAtRank(0)->initial();
  p01 = optimizedPath3->pathAtRank(0)->end();
  p10 = optimizedPath3->pathAtRank(1)->initial();
  p11 = optimizedPath3->pathAtRank(1)->end();
  p20 = optimizedPath3->pathAtRank(2)->initial();
  p21 = optimizedPath3->pathAtRank(2)->end();
  p30 = optimizedPath3->pathAtRank(3)->initial();
  p31 = optimizedPath3->pathAtRank(3)->end();
  value_type L;
  vector_t v00(robot->numberDof()), v01(robot->numberDof()),
      v10(robot->numberDof()), v11(robot->numberDof()), v20(robot->numberDof()),
      v21(robot->numberDof()), v30(robot->numberDof()), v31(robot->numberDof());
  L = optimizedPath3->pathAtRank(0)->length();
  optimizedPath3->pathAtRank(0)->derivative(v00, 0, 1);
  optimizedPath3->pathAtRank(0)->derivative(v01, L, 1);
  L = optimizedPath3->pathAtRank(1)->length();
  optimizedPath3->pathAtRank(1)->derivative(v10, 0, 1);
  optimizedPath3->pathAtRank(1)->derivative(v11, L, 1);
  L = optimizedPath3->pathAtRank(2)->length();
  optimizedPath3->pathAtRank(2)->derivative(v20, 0, 1);
  optimizedPath3->pathAtRank(2)->derivative(v21, L, 1);
  L = optimizedPath3->pathAtRank(3)->length();
  optimizedPath3->pathAtRank(3)->derivative(v30, 0, 1);
  optimizedPath3->pathAtRank(3)->derivative(v31, L, 1);

  r1 << -0.520833333333333, 0, 1, 0;
  r2 << 0, 0, 1, 0;
  r3 << 0.520833333333333, 0, 1, 0;

  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - r1).norm() < 1e-10);
  BOOST_CHECK((p10 - r1).norm() < 1e-10);
  BOOST_CHECK((p11 - r2).norm() < 1e-10);
  BOOST_CHECK((p20 - r2).norm() < 1e-10);
  BOOST_CHECK((p21 - r3).norm() < 1e-10);
  BOOST_CHECK((p30 - r3).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);

  vector_t v0(robot->numberDof()), v1(robot->numberDof()),
      v2(robot->numberDof()), v3(robot->numberDof()), v4(robot->numberDof());
  v0 << 0, 0, 0;
  v1 << 0.562800733001311, 0, 0;
  v2 << 0.643200837715785, 0, 0;
  v3 << 0.562800733001311, 0, 0;
  v4 << 0, 0, 0;

  // Continuity of the first derivative
  BOOST_CHECK((v00 - v0).norm() < 1e-10);
  BOOST_CHECK((v01 - v1).norm() < 1e-10);
  BOOST_CHECK((v10 - v1).norm() < 1e-10);
  BOOST_CHECK((v11 - v2).norm() < 1e-10);
  BOOST_CHECK((v20 - v2).norm() < 1e-10);
  BOOST_CHECK((v21 - v3).norm() < 1e-10);
  BOOST_CHECK((v30 - v3).norm() < 1e-10);
  BOOST_CHECK((v31 - v4).norm() < 1e-10);

  // Test spline of degree 5
  p00 = optimizedPath5->pathAtRank(0)->initial();
  p01 = optimizedPath5->pathAtRank(0)->end();
  p10 = optimizedPath5->pathAtRank(1)->initial();
  p11 = optimizedPath5->pathAtRank(1)->end();
  p20 = optimizedPath5->pathAtRank(2)->initial();
  p21 = optimizedPath5->pathAtRank(2)->end();
  p30 = optimizedPath5->pathAtRank(3)->initial();
  p31 = optimizedPath5->pathAtRank(3)->end();

  r1 << -0.553756964312603, 0, 1, 0;
  r2 << 0, 0, 1, 0;
  r3 << 0.553756964312603, 0, 1, 0;

  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - r1).norm() < 1e-10);
  BOOST_CHECK((p10 - r1).norm() < 1e-10);
  BOOST_CHECK((p11 - r2).norm() < 1e-10);
  BOOST_CHECK((p20 - r2).norm() < 1e-10);
  BOOST_CHECK((p21 - r3).norm() < 1e-10);
  BOOST_CHECK((p30 - r3).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);

  L = optimizedPath5->pathAtRank(0)->length();
  optimizedPath5->pathAtRank(0)->derivative(v00, 0, 1);
  optimizedPath5->pathAtRank(0)->derivative(v01, L, 1);
  L = optimizedPath5->pathAtRank(1)->length();
  optimizedPath5->pathAtRank(1)->derivative(v10, 0, 1);
  optimizedPath5->pathAtRank(1)->derivative(v11, L, 1);
  L = optimizedPath5->pathAtRank(2)->length();
  optimizedPath5->pathAtRank(2)->derivative(v20, 0, 1);
  optimizedPath5->pathAtRank(2)->derivative(v21, L, 1);
  L = optimizedPath5->pathAtRank(3)->length();
  optimizedPath5->pathAtRank(3)->derivative(v30, 0, 1);
  optimizedPath5->pathAtRank(3)->derivative(v31, L, 1);

  v1 << 0.626154708000684, 0, 0;
  v2 << 0.740925524548374, 0, 0;
  v3 << 0.626154708000684, 0, 0;

  // Continuity of the first derivative
  BOOST_CHECK((v00 - v0).norm() < 1e-10);
  BOOST_CHECK((v01 - v1).norm() < 1e-10);
  BOOST_CHECK((v10 - v1).norm() < 1e-10);
  BOOST_CHECK((v11 - v2).norm() < 1e-10);
  BOOST_CHECK((v20 - v2).norm() < 1e-10);
  BOOST_CHECK((v21 - v3).norm() < 1e-10);
  BOOST_CHECK((v30 - v3).norm() < 1e-10);
  BOOST_CHECK((v31 - v4).norm() < 1e-10);
}

//
// Same as previously, but with a cylindrical obstacle
//
BOOST_AUTO_TEST_CASE(spline_optimization_obstacle) {
  Eigen::IOFormat eigenFmt(Eigen::FullPrecision);
  ::hpp::debug::setVerbosityLevel(100);
  DevicePtr_t robot = createRobot();
  Configuration_t q0(robot->configSize());
  Configuration_t q1(robot->configSize());
  Configuration_t q2(robot->configSize());
  Configuration_t q3(robot->configSize());
  Configuration_t q4(robot->configSize());
  value_type s = sqrt(2) / 2;
  value_type L, t, cost, dt = 0.01;
  q0(0) = -1;
  q0(1) = 0;
  q0(2) = 0;
  q0(3) = 1;
  q1(0) = -s;
  q1(1) = s;
  q1(2) = s;
  q1(3) = s;
  q2(0) = 0;
  q2(1) = 1;
  q2(2) = 1;
  q2(3) = 0;
  q3(0) = s;
  q3(1) = s;
  q3(2) = s;
  q3(3) = -s;
  q4(0) = 1;
  q4(1) = 0;
  q4(2) = 0;
  q4(3) = -1;

  ProblemPtr_t problem = Problem::create(robot);
  PathValidationPtr_t pv(continuousValidation::Dichotomy::create(robot, 0));
  problem->pathValidation(pv);
  hpp::pinocchio::Model obstacleRModel;
  hpp::pinocchio::GeomModelPtr_t obstacleModel(new hpp::pinocchio::GeomModel);
  hpp::pinocchio::GeomDataPtr_t obstacleData(
      new hpp::pinocchio::GeomData(*obstacleModel));
  pinocchio::GeometryObject::CollisionGeometryPtr cylinder(
      new hpp::fcl::Cylinder(0.2, 0.2));
  matrix3_t I3(matrix3_t::Identity(3, 3));
  vector3_t T;
  T << -.2, 0, 0;
  pinocchio::SE3 pose(I3, T);
  ::pinocchio::GeomIndex id = obstacleModel->addGeometryObject(
      ::pinocchio::GeometryObject("obstacle", 0, cylinder, pose),
      obstacleRModel);
  obstacleData->oMg.resize(obstacleModel->ngeoms);
  obstacleData->oMg[id] = obstacleModel->geometryObjects[id].placement;
  CollisionObjectPtr_t object(
      new CollisionObject(obstacleModel, obstacleData, id));
  problem->addObstacle(object);

  SteeringMethodPtr_t sm = problem->steeringMethod();
  PathVectorPtr_t path =
      PathVector::create(robot->configSize(), robot->numberDof());
  path->appendPath((*sm)(q0, q1));
  path->appendPath((*sm)(q1, q2));
  path->appendPath((*sm)(q2, q3));
  path->appendPath((*sm)(q3, q4));

  problem->setParameter("SplineGradientBased/alphaInit",
                        hpp::core::Parameter(.5));
  PathOptimizerPtr_t pathOptimizer1(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 1>::create(
          problem));
  PathOptimizerPtr_t pathOptimizer3(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 3>::create(
          problem));
  PathOptimizerPtr_t pathOptimizer5(
      pathOptimization::SplineGradientBased<path::BernsteinBasis, 5>::create(
          problem));
  PathVectorPtr_t optimizedPath1(pathOptimizer1->optimize(path));
  BOOST_CHECK(optimizedPath1->numberPaths() == 4);
  Configuration_t p00 = optimizedPath1->pathAtRank(0)->initial();
  Configuration_t p01 = optimizedPath1->pathAtRank(0)->end();
  Configuration_t p10 = optimizedPath1->pathAtRank(1)->initial();
  Configuration_t p11 = optimizedPath1->pathAtRank(1)->end();
  Configuration_t p20 = optimizedPath1->pathAtRank(2)->initial();
  Configuration_t p21 = optimizedPath1->pathAtRank(2)->end();
  Configuration_t p30 = optimizedPath1->pathAtRank(3)->initial();
  Configuration_t p31 = optimizedPath1->pathAtRank(3)->end();
  Configuration_t r1(robot->configSize());
  Configuration_t r2(robot->configSize());
  Configuration_t r3(robot->configSize());
  // Test spline of degree 1
  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - p10).norm() < 1e-10);
  BOOST_CHECK((p11 - p20).norm() < 1e-10);
  BOOST_CHECK((p21 - p30).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);
  L = optimizedPath1->length();
  t = 0;
  cost = 0;
  while (t <= L) {
    vector3_t v;
    optimizedPath1->derivative(v, t, 1);
    cost += dt * v.norm();
    t += dt;
  }
  BOOST_CHECK(cost <= 4.);
  // Test spline of degree 3
  PathVectorPtr_t optimizedPath3(pathOptimizer3->optimize(path));
  BOOST_CHECK(optimizedPath3->numberPaths() == 4);
  p00 = optimizedPath3->pathAtRank(0)->initial();
  p01 = optimizedPath3->pathAtRank(0)->end();
  p10 = optimizedPath3->pathAtRank(1)->initial();
  p11 = optimizedPath3->pathAtRank(1)->end();
  p20 = optimizedPath3->pathAtRank(2)->initial();
  p21 = optimizedPath3->pathAtRank(2)->end();
  p30 = optimizedPath3->pathAtRank(3)->initial();
  p31 = optimizedPath3->pathAtRank(3)->end();
  vector_t v00(robot->numberDof()), v01(robot->numberDof()),
      v10(robot->numberDof()), v11(robot->numberDof()), v20(robot->numberDof()),
      v21(robot->numberDof()), v30(robot->numberDof()), v31(robot->numberDof());
  L = optimizedPath3->pathAtRank(0)->length();
  optimizedPath3->pathAtRank(0)->derivative(v00, 0, 1);
  optimizedPath3->pathAtRank(0)->derivative(v01, L, 1);
  L = optimizedPath3->pathAtRank(1)->length();
  optimizedPath3->pathAtRank(1)->derivative(v10, 0, 1);
  optimizedPath3->pathAtRank(1)->derivative(v11, L, 1);
  L = optimizedPath3->pathAtRank(2)->length();
  optimizedPath3->pathAtRank(2)->derivative(v20, 0, 1);
  optimizedPath3->pathAtRank(2)->derivative(v21, L, 1);
  L = optimizedPath3->pathAtRank(3)->length();
  optimizedPath3->pathAtRank(3)->derivative(v30, 0, 1);
  optimizedPath3->pathAtRank(3)->derivative(v31, L, 1);

  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - p10).norm() < 1e-10);
  BOOST_CHECK((p11 - p20).norm() < 1e-10);
  BOOST_CHECK((p21 - p30).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);

  vector_t v0(robot->numberDof()), v1(robot->numberDof()),
      v2(robot->numberDof()), v3(robot->numberDof()), v4(robot->numberDof());
  v0 << 0, 0, 0;
  v4 << 0, 0, 0;

  // Continuity of the first derivative
  BOOST_CHECK((v00 - v0).norm() < 1e-10);
  BOOST_CHECK((v01 - v10).norm() < 1e-10);
  BOOST_CHECK((v11 - v20).norm() < 1e-10);
  BOOST_CHECK((v21 - v30).norm() < 1e-10);
  BOOST_CHECK((v31 - v4).norm() < 1e-10);

  L = optimizedPath3->length();
  t = 0, dt = 0.01;
  cost = 0;
  while (t <= L) {
    vector3_t v;
    optimizedPath3->derivative(v, t, 1);
    cost += dt * v.norm();
    t += dt;
  }
  BOOST_CHECK(cost <= 4.);

  // Test spline of degree 5
  PathVectorPtr_t optimizedPath5(pathOptimizer5->optimize(path));
  BOOST_CHECK(optimizedPath5->numberPaths() == 4);
  p00 = optimizedPath5->pathAtRank(0)->initial();
  p01 = optimizedPath5->pathAtRank(0)->end();
  p10 = optimizedPath5->pathAtRank(1)->initial();
  p11 = optimizedPath5->pathAtRank(1)->end();
  p20 = optimizedPath5->pathAtRank(2)->initial();
  p21 = optimizedPath5->pathAtRank(2)->end();
  p30 = optimizedPath5->pathAtRank(3)->initial();
  p31 = optimizedPath5->pathAtRank(3)->end();

  // Continuity
  BOOST_CHECK((p00 - q0).norm() < 1e-10);
  BOOST_CHECK((p01 - p10).norm() < 1e-10);
  BOOST_CHECK((p11 - p20).norm() < 1e-10);
  BOOST_CHECK((p21 - p30).norm() < 1e-10);
  BOOST_CHECK((p31 - q4).norm() < 1e-10);

  L = optimizedPath5->pathAtRank(0)->length();
  optimizedPath5->pathAtRank(0)->derivative(v00, 0, 1);
  optimizedPath5->pathAtRank(0)->derivative(v01, L, 1);
  L = optimizedPath5->pathAtRank(1)->length();
  optimizedPath5->pathAtRank(1)->derivative(v10, 0, 1);
  optimizedPath5->pathAtRank(1)->derivative(v11, L, 1);
  L = optimizedPath5->pathAtRank(2)->length();
  optimizedPath5->pathAtRank(2)->derivative(v20, 0, 1);
  optimizedPath5->pathAtRank(2)->derivative(v21, L, 1);
  L = optimizedPath5->pathAtRank(3)->length();
  optimizedPath5->pathAtRank(3)->derivative(v30, 0, 1);
  optimizedPath5->pathAtRank(3)->derivative(v31, L, 1);

  // Continuity of the first derivative
  BOOST_CHECK((v00 - v0).norm() < 1e-10);
  BOOST_CHECK((v01 - v10).norm() < 1e-10);
  BOOST_CHECK((v11 - v20).norm() < 1e-10);
  BOOST_CHECK((v21 - v30).norm() < 1e-10);
  BOOST_CHECK((v31 - v4).norm() < 1e-10);

  L = optimizedPath5->length();
  t = 0, dt = 0.01;
  cost = 0;
  while (t <= L) {
    vector3_t v;
    optimizedPath5->derivative(v, t, 1);
    cost += dt * v.norm();
    t += dt;
  }
  BOOST_CHECK(cost <= 4.);
}
BOOST_AUTO_TEST_SUITE_END()
