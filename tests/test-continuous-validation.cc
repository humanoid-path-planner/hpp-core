// Copyright (c) 2017, LAAS-CNRS
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

#define BOOST_TEST_MODULE ContinuousValidation
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/test/included/unit_test.hpp>
#include <pinocchio/fwd.hpp>
namespace bpt = boost::posix_time;

#ifdef _OPENMP
#include <omp.h>
#endif

#include <hpp/core/collision-validation.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>

using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

using hpp::pinocchio::urdf::loadModel;
using hpp::pinocchio::urdf::loadModelFromString;

using hpp::core::CollisionValidation;
using hpp::core::Configuration_t;
using hpp::core::ConfigurationShooterPtr_t;
using hpp::core::ConfigValidationPtr_t;
using hpp::core::matrix_t;
using hpp::core::PathPtr_t;
using hpp::core::PathValidationPtr_t;
using hpp::core::PathValidationReportPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::size_type;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::ValidationReportPtr_t;
using hpp::core::value_type;
using hpp::core::vector_t;
using hpp::core::configurationShooter::Uniform;
using hpp::core::continuousCollisionChecking::Progressive;
using hpp::core::continuousValidation::Dichotomy;
using hpp::core::continuousValidation::DichotomyPtr_t;
using hpp::core::pathValidation::createDiscretizedCollisionChecking;
using hpp::core::steeringMethod::Straight;

namespace steeringMethod = hpp::core::steeringMethod;

static size_type i1 = 0, n1 = 100;
static size_type i2 = 0, n2 = 10;

BOOST_AUTO_TEST_SUITE(test_hpp_core)

matrix_t generateRandomConfig(const DevicePtr_t& robot, size_type n) {
  // Create configuration shooter
  ConfigurationShooterPtr_t shooter(Uniform::create(robot));
  matrix_t m(n, robot->configSize());
  for (size_type i = 0; i < n; ++i) {
    m.row(i) = shooter->shoot();
  }
  return m;
}

matrix_t generateRandomVelocities(const DevicePtr_t& robot, size_type n) {
  // Create configuration shooter
  ConfigurationShooterPtr_t shooter(Uniform::create(robot));
  matrix_t m(n, robot->numberDof());
  for (size_type i = 0; i < n; ++i) {
    m.row(i) = vector_t::Random(robot->numberDof());
  }
  return m;
}

void generate_random_numbers() {
  // Load robot model (ur5)
  DevicePtr_t robot(Device::create("ur5"));
  loadModel(robot, 0, "", "anchor",
            "package://example-robot-data/robots/ur_description/"
            "urdf/ur5_joint_limited_robot.urdf",
            "package://example-robot-data/robots/ur_description/"
            "srdf/ur5_joint_limited_robot.srdf");
  matrix_t rand1 = generateRandomConfig(robot, 2 * (n1 + n2));
  matrix_t rand2 = generateRandomVelocities(robot, 2 * n2);

  Eigen::IOFormat f1(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ",
                     "", "", "", ";\n");
  Eigen::IOFormat f2(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ",
                     "", "", "", ";\n");
  std::cout << "static matrix_t m1 (" << rand1.rows() << ", " << rand1.cols()
            << "); m1 << " << rand1.format(f1);
  std::cout << "static matrix_t m2 (" << rand2.rows() << ", " << rand2.cols()
            << "); m2 << " << rand2.format(f2);
}

#if 0
BOOST_AUTO_TEST_CASE (random)
{
  generate_random_numbers ();
}
#else

static matrix_t m1(220, 6);
static matrix_t m2(20, 6);

BOOST_AUTO_TEST_CASE(continuous_validation_straight) {
#include "../tests/random-numbers.hh"
  i1 = 0;
  i2 = 0;

  // Load robot model (ur5)
  DevicePtr_t robot(Device::create("ur5"));
  loadModel(robot, 0, "", "anchor",
            "package://example-robot-data/robots/ur_description/"
            "urdf/ur5_joint_limited_robot.urdf",
            "package://example-robot-data/robots/ur_description/"
            "srdf/ur5_joint_limited_robot.srdf");
  robot->numberDeviceData(4);

  // create steering method
  ProblemPtr_t problem = Problem::create(robot);
  SteeringMethodPtr_t sm(Straight::create(problem));

  // create path validation objects
  PathValidationPtr_t dichotomy(Dichotomy::create(robot, 0));
  PathValidationPtr_t progressive(Progressive::create(robot, 0.001));
  PathValidationPtr_t discretized(
      createDiscretizedCollisionChecking(robot, 0.05));
  // create configuration validation instance
  ConfigValidationPtr_t configValidation(CollisionValidation::create(robot));
  ValidationReportPtr_t collisionReport;
  //  create random paths and test them with different validation instances
  Configuration_t q1(robot->configSize()), q2(robot->configSize());
  bpt::ptime t0 = bpt::microsec_clock::local_time();
  for (size_type i = 0; i < n1; ++i) {
    {
      q1 = m1.row(i1);
      ++i1;
      q2 = m1.row(i1);
      ++i1;
    }
    PathValidationReportPtr_t report1;
    PathValidationReportPtr_t report2;
    PathValidationReportPtr_t report3;
    ValidationReportPtr_t report4;
    ValidationReportPtr_t report5;
    ValidationReportPtr_t report6;
    PathPtr_t path((*sm)(q1, q2));
    bool res4(discretized->validate(q1, report4));
    bool res5(progressive->validate(q1, report5));
    bool res6(dichotomy->validate(q1, report6));
    PathPtr_t validPart;
    if (configValidation->validate(q1, collisionReport)) {
      bool res1(discretized->validate(path, false, validPart, report1));
      bool res2(progressive->validate(path, false, validPart, report2));
      bool res3(dichotomy->validate(path, false, validPart, report3));

      // Check that PathValidation::validate(ConfigurationIn_t,...) returns
      // the same result as config validation.
      if (!res4) {
        std::cout << "q=" << hpp::pinocchio::displayConfig(q1) << std::endl;
        std::cout << "report 4: " << *report4 << std::endl;
      }
      if (!res5) {
        std::cout << "q=" << hpp::pinocchio::displayConfig(q1) << std::endl;
        std::cout << "report 5: " << *report5 << std::endl;
      }
      if (!res6) {
        std::cout << "q=" << hpp::pinocchio::displayConfig(q1) << std::endl;
        std::cout << "report 6: " << *report6 << std::endl;
      }
      BOOST_CHECK(res4);
      BOOST_CHECK(res5);
      BOOST_CHECK(res6);
      if (!res1) {
        BOOST_CHECK(!res2);
        BOOST_CHECK(!res3);
        if (res2) {
          hppDout(error, "Progressive failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
        if (res3) {
          hppDout(error, "Dichotomy failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
      }
      if (res1) {
        BOOST_CHECK(res2);
        BOOST_CHECK(res3);
        if (!res2) {
          hppDout(info,
                  "Progressive found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report2);
        }
        if (!res3) {
          hppDout(info,
                  "Dichotomy found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report3);
        }
      }
    } else {
      // Check that PathValidation::validate(ConfigurationIn_t,...) returns
      // the same result as config validation.
      if (res4 || res5 || res6) {
        std::cout << "q=" << hpp::pinocchio::displayConfig(q1) << std::endl;
        std::cout << "collisionReport: " << *collisionReport << std::endl;
      }
      BOOST_CHECK(!res4);
      BOOST_CHECK(!res5);
      BOOST_CHECK(!res6);
    }
    if (configValidation->validate(q2, collisionReport)) {
      bool res1(discretized->validate(path, true, validPart, report1));
      bool res2(progressive->validate(path, true, validPart, report2));
      bool res3(dichotomy->validate(path, true, validPart, report3));

      if (!res1) {
        BOOST_CHECK(!res2);
        BOOST_CHECK(!res3);
        if (res2) {
          hppDout(error, "Progressive failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
        if (res3) {
          hppDout(error, "Dichotomy failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
      }
      if (res1) {
        if (!res2) {
          hppDout(info,
                  "Progressive found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report2);
        }
        if (!res3) {
          hppDout(info,
                  "Dichotomy found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report3);
        }
      }
    }
  }
  bpt::ptime t1 = bpt::microsec_clock::local_time();
  BOOST_TEST_MESSAGE("Total time: " << (t1 - t0).total_milliseconds() << "ms");
  // delete problem
}

template <typename SplineSteeringMethod>
void test_spline_steering_method() {
#include "../tests/random-numbers.hh"
  i1 = 0;
  i2 = 0;

  // Load robot model (ur5)
  DevicePtr_t robot(Device::create("ur5"));
  loadModel(robot, 0, "", "anchor",
            "package://example-robot-data/robots/ur_description/"
            "urdf/ur5_joint_limited_robot.urdf",
            "package://example-robot-data/robots/ur_description/"
            "srdf/ur5_joint_limited_robot.srdf");
  robot->numberDeviceData(4);

  // create steering method
  ProblemPtr_t problem = Problem::create(robot);
  typename SplineSteeringMethod::Ptr_t sm(
      SplineSteeringMethod::create(problem));

  // create path validation objects
  // PathValidationPtr_t dichotomy (Dichotomy::create (robot, 0));
  PathValidationPtr_t progressive(Progressive::create(robot, 0.01));
  PathValidationPtr_t discretized(
      createDiscretizedCollisionChecking(robot, 0.05));
  // create configuration validation instance
  ConfigValidationPtr_t configValidation(CollisionValidation::create(robot));
  ValidationReportPtr_t collisionReport;

  std::vector<int> orders(1, 1);
  //  create random paths and test them with different validation instances
  Configuration_t q1(robot->configSize()), q2(robot->configSize());
  vector_t v1(robot->numberDof()), v2(robot->numberDof());
  int Nthreads = 1;

  bpt::ptime t0 = bpt::microsec_clock::local_time();
#pragma omp parallel for
  for (size_type i = 0; i < n2; ++i) {
#ifdef _OPENMP
    Nthreads = omp_get_num_threads();
#endif
#pragma omp critical
    {
      q1 = m1.row(i1++);
      q2 = m1.row(i1++);
      v1 = m2.row(i2++);
      v2 = m2.row(i2++);
    }
    PathValidationReportPtr_t report1;
    PathValidationReportPtr_t report2;
    PathValidationReportPtr_t report3;
    PathPtr_t path(sm->steer(q1, orders, v1, q2, orders, v2));
    PathPtr_t validPart;
    if (configValidation->validate(q1, collisionReport)) {
      bool res1(discretized->validate(path, false, validPart, report1));
      bool res2(progressive->validate(path, false, validPart, report2));
      // bool res3 (dichotomy->validate (path, false, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK(!res2);
        // BOOST_CHECK (!res3);
        if (res2) {
          hppDout(error, "Progressive failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
        /*if (res3) {
          hppDout (error, "Dichotomy failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }*/
      }
#pragma omp critical
      if (res1) {
        BOOST_CHECK(res2);
        if (!res2) {
          hppDout(info,
                  "Progressive found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report2);
        }
        /*if (!res3) {
          hppDout (info, "Dichotomy found a collision where discretized did "
                     "not for q1 = " << q1.transpose () << ", q2 = "
                   << q2.transpose ());
          hppDout (info, *report3);
        }*/
      }
    }
    if (configValidation->validate(q2, collisionReport)) {
      bool res1(discretized->validate(path, true, validPart, report1));
      bool res2(progressive->validate(path, true, validPart, report2));
      // bool res3 (dichotomy->validate (path, true, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK(!res2);
        // BOOST_CHECK (!res3);
        if (res2) {
          hppDout(error, "Progressive failed to detect collision for q1="
                             << q1.transpose() << ", q2=" << q2.transpose());
          hppDout(error, *report1);
        }
        /*if (res3) {
          hppDout (error, "Dichotomy failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }*/
      }
#pragma omp critical
      if (res1) {
        if (!res2) {
          hppDout(info,
                  "Progressive found a collision where discretized did "
                  "not for q1 = "
                      << q1.transpose() << ", q2 = " << q2.transpose());
          hppDout(info, *report2);
        }
        /*if (!res3) {
          hppDout (info, "Dichotomy found a collision where discretized did "
                     "not for q1 = " << q1.transpose () << ", q2 = "
                   << q2.transpose ());
          hppDout (info, *report3);
        }*/
      }
    }
  }
  bpt::ptime t1 = bpt::microsec_clock::local_time();
  BOOST_TEST_MESSAGE("Total time (nthreads "
                     << Nthreads << "): " << (t1 - t0).total_milliseconds()
                     << "ms");
  // delete problem
}

BOOST_AUTO_TEST_CASE(continuous_validation_spline) {
  test_spline_steering_method<
      hpp::core::steeringMethod::Spline<hpp::core::path::BernsteinBasis, 3> >();
}
#endif

std::string avoid_infinite_loop_urdf_string = R"(
<robot name="test">
<link name="base">
  <collision>
  <geometry><box size="1 1 1"/></geometry>
  </collision>
</link>
<link name="middle"/>
<link name="tail">
  <collision>
  <geometry><sphere radius="0.5"/></geometry>
  </collision>
</link>
<joint name="joint1" type="prismatic">
  <axis xyz="1 0 0"/>
  <parent link="base"/>
  <child link="middle"/>
  <limit effort="30" velocity="1.0" lower="-3" upper="3" />
</joint>
<joint name="joint2" type="prismatic">
  <axis xyz="0 1 0"/>
  <parent link="middle"/>
  <child link="tail"/>
  <limit effort="30" velocity="1.0" lower="-3" upper="3" />
</joint>
</robot>
)";

BOOST_AUTO_TEST_CASE(avoid_infinite_loop_straight) {
  DevicePtr_t robot(Device::create("test"));
  loadModelFromString(robot, 0, "", "anchor", avoid_infinite_loop_urdf_string,
                      "");

  // create steering method
  ProblemPtr_t problem = Problem::create(robot);
  SteeringMethodPtr_t sm(Straight::create(problem));

  // create path validation objects
  DichotomyPtr_t dichotomy(Dichotomy::create(robot, 0));
  dichotomy->distanceLowerBoundThreshold(0.001);

  Configuration_t q1(robot->configSize()), q2(robot->configSize());
  for (hpp::core::value_type min_dist : {0.00101, 0.00099, 1e-6, 0.0}) {
    q1 << -1, 1 + min_dist;
    q2 << 1.1, 1 + min_dist;
    PathPtr_t path((*sm)(q1, q2));

    PathPtr_t validPart;
    PathValidationReportPtr_t report;
    bool res(dichotomy->validate(path, false, validPart, report));

    BOOST_TEST_MESSAGE(min_dist << " - " << res);
    if (report) BOOST_TEST_MESSAGE(*report);
    // TODO we want to check the number of iterations.
  }
}

BOOST_AUTO_TEST_CASE(avoid_infinite_loop_spline) {
  DevicePtr_t robot(Device::create("test"));
  loadModelFromString(robot, 0, "", "anchor", avoid_infinite_loop_urdf_string,
                      "");

  // create steering method
  ProblemPtr_t problem = Problem::create(robot);
  typedef steeringMethod::Spline<hpp::core::path::BernsteinBasis, 3> SMSpline_t;
  SMSpline_t::Ptr_t sm(SMSpline_t::create(problem));
  Configuration_t q1(robot->configSize()), q2(robot->configSize());
  PathPtr_t path;
  bool ok;

  // Calculate shift
  std::vector<int> order = {1};
  matrix_t D1(robot->numberDof(), 1), D2(robot->numberDof(), 1);

  q1 << -1, 0;
  q2 << 1.1, 0;
  D1 << 0, -1;
  D2 << 0, 1;
  path = sm->steer(q1, order, D1, q2, order, D2, 1.0);
  BOOST_REQUIRE_EQUAL(path->length(), 1.0);
  value_type shift = -path->eval(0.5, ok)[1];

  // create path validation objects
  DichotomyPtr_t dichotomy(Dichotomy::create(robot, 0));
  dichotomy->distanceLowerBoundThreshold(0.001);

  for (hpp::core::value_type min_dist : {0.00101, 0.00099, 1e-6, 0.0}) {
    q1 << -1, shift + 1 + min_dist;
    q2 << 1.1, shift + 1 + min_dist;
    path = sm->steer(q1, order, D1, q2, order, D2, 1.0);

    PathPtr_t validPart;
    PathValidationReportPtr_t report;
    bool res(dichotomy->validate(path, false, validPart, report));

    BOOST_TEST_MESSAGE(min_dist << " - " << res);
    if (report) BOOST_TEST_MESSAGE(*report);
    // TODO we want to check the number of iterations.
  }
}

BOOST_AUTO_TEST_SUITE_END()
