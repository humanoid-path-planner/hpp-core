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

#define BOOST_TEST_MODULE ContinuousValidation

#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
namespace bpt = boost::posix_time;

#ifdef _OPENMP
# include <omp.h>
#endif

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/path/spline.hh>

using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

using hpp::pinocchio::urdf::loadRobotModel;

using hpp::core::matrix_t;
using hpp::core::vector_t;
using hpp::core::size_type;
using hpp::core::configurationShooter::Uniform;
using hpp::core::CollisionValidation;
using hpp::core::Configuration_t;
using hpp::core::ConfigurationShooterPtr_t;
using hpp::core::ConfigValidationPtr_t;
using hpp::core::continuousCollisionChecking::Dichotomy;
using hpp::core::continuousCollisionChecking::Progressive;
using hpp::core::pathValidation::createDiscretizedCollisionChecking;
using hpp::core::PathPtr_t;
using hpp::core::PathValidationPtr_t;
using hpp::core::PathValidationReportPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::steeringMethod::Straight;
using hpp::core::ValidationReportPtr_t;

static size_type i1 = 0, n1 = 100;
static size_type i2 = 0, n2 = 10;

BOOST_AUTO_TEST_SUITE (test_hpp_core)

matrix_t generateRandomConfig (const DevicePtr_t& robot, size_type n)
{
  // Create configuration shooter
  ConfigurationShooterPtr_t shooter (Uniform::create (robot));
  matrix_t m (n, robot->configSize ());
  for (size_type i=0; i<n; ++i) {
    m.row (i) = *(shooter->shoot ());
  }
  return m;
}

matrix_t generateRandomVelocities (const DevicePtr_t& robot, size_type n)
{
  // Create configuration shooter
  ConfigurationShooterPtr_t shooter (Uniform::create (robot));
  matrix_t m (n, robot->numberDof ());
  for (size_type i=0; i<n; ++i) {
    m.row (i) = vector_t::Random (robot->numberDof());
  }
  return m;
}

void generate_random_numbers ()
{
  // Load robot model (ur5)
  DevicePtr_t robot (Device::create ("ur5"));
  loadRobotModel (robot, "anchor", "ur_description", "ur5_joint_limited_robot",
                  "", "");
  matrix_t rand1 = generateRandomConfig (robot, 2*(n1+n2));
  matrix_t rand2 = generateRandomVelocities (robot, 2*n2);

  Eigen::IOFormat f1 (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                          ", ", "", "", "", ";\n");
  Eigen::IOFormat f2 (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                          ", ", "", "", "", ";\n");
  std::cout << "static matrix_t m1 (" << rand1.rows () << ", " << rand1.cols ()
            << "); m1 << " << rand1.format (f1);
  std::cout << "static matrix_t m2 (" << rand2.rows () << ", " << rand2.cols ()
            << "); m2 << " << rand2.format (f2);
}

#if 0
BOOST_AUTO_TEST_CASE (random)
{
  generate_random_numbers ();
}
#else

static matrix_t m1 (220, 6);
static matrix_t m2 (20, 6);

BOOST_AUTO_TEST_CASE (continuous_validation_straight)
{

  #include "../tests/random-numbers.hh"
  i1 = 0;
  i2 = 0;

  // Load robot model (ur5)
  DevicePtr_t robot (Device::create ("ur5"));
  loadRobotModel (robot, "anchor", "ur_description", "ur5_joint_limited_robot",
                  "", "");
  robot->numberDeviceData (4);

  // create steering method
  Problem problem (robot);
  SteeringMethodPtr_t sm (Straight::create (problem));

  // create path validation objects
  PathValidationPtr_t dichotomy (Dichotomy::create (robot, 0));
  PathValidationPtr_t progressive (Progressive::create (robot, 0.001));
  PathValidationPtr_t discretized (createDiscretizedCollisionChecking
                                   (robot, 0.05));
  // create configuration validation instance
  ConfigValidationPtr_t configValidation (CollisionValidation::create (robot));
  ValidationReportPtr_t collisionReport;
  //  create random paths and test them with different validation instances
  Configuration_t q1 (robot->configSize()),
                  q2 (robot->configSize());
  bpt::ptime t0 = bpt::microsec_clock::local_time();
  int Nthreads = 1;
#pragma omp parallel for
  for (size_type i=0; i<n1; ++i) {
#ifdef _OPENMP
    Nthreads = omp_get_num_threads();
#endif
#pragma omp critical
    {
      q1 = m1.row (i1); ++i1;
      q2 = m1.row (i1); ++i1;
    }
    PathValidationReportPtr_t report1;
    PathValidationReportPtr_t report2;
    PathValidationReportPtr_t report3;
    PathPtr_t path ((*sm) (q1, q2));
    PathPtr_t validPart;
    if (configValidation->validate (q1, collisionReport)) {
      bool res1 (discretized->validate (path, false, validPart, report1));
      bool res2 (progressive->validate  (path, false, validPart, report2));
      bool res3 (dichotomy->validate (path, false, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK (!res2);
        BOOST_CHECK (!res3);
        if (res2) {
          hppDout (error, "Progressive failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }
        if (res3) {
          hppDout (error, "Dichotomy failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }
      }
#pragma omp critical
      if (res1) {
        BOOST_CHECK (res2);
        BOOST_CHECK (res3);
        if (!res2) {
          hppDout (info, "Progressive found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report2);
        }
        if (!res3) {
          hppDout (info, "Dichotomy found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report3);
        }
      }
    }
    if (configValidation->validate (q2, collisionReport)) {
      bool res1 (discretized->validate (path, true, validPart, report1));
      bool res2 (progressive->validate  (path, true, validPart, report2));
      bool res3 (dichotomy->validate (path, true, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK (!res2);
        BOOST_CHECK (!res3);
        if (res2) {
          hppDout (error, "Progressive failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }
        if (res3) {
          hppDout (error, "Dichotomy failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }
      }
#pragma omp critical
      if (res1) {
        if (!res2) {
          hppDout (info, "Progressive found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report2);
        }
        if (!res3) {
          hppDout (info, "Dichotomy found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report3);
        }
      }
    }
  }
  bpt::ptime t1 = bpt::microsec_clock::local_time();
  BOOST_MESSAGE ("Total time (nthreads " << Nthreads << "): " << (t1-t0).total_milliseconds() << "ms");
  // delete problem
}

template <typename SplineSteeringMethod> void test_spline_steering_method ()
{
  #include "../tests/random-numbers.hh"
  i1 = 0;
  i2 = 0;

  // Load robot model (ur5)
  DevicePtr_t robot (Device::create ("ur5"));
  loadRobotModel (robot, "anchor", "ur_description", "ur5_joint_limited_robot",
                  "", "");
  robot->numberDeviceData (4);

  // create steering method
  Problem problem (robot);
  typename SplineSteeringMethod::Ptr_t sm (SplineSteeringMethod::create (problem));

  // create path validation objects
  // PathValidationPtr_t dichotomy (Dichotomy::create (robot, 0));
  PathValidationPtr_t progressive (Progressive::create (robot, 0.01));
  PathValidationPtr_t discretized (createDiscretizedCollisionChecking
                                   (robot, 0.05));
  // create configuration validation instance
  ConfigValidationPtr_t configValidation (CollisionValidation::create (robot));
  ValidationReportPtr_t collisionReport;

  std::vector<int> orders (1, 1);
  //  create random paths and test them with different validation instances
  Configuration_t q1 (robot->configSize()),
                  q2 (robot->configSize());
  vector_t v1 (robot->numberDof()),
           v2 (robot->numberDof());
  int Nthreads = 1;

  bpt::ptime t0 = bpt::microsec_clock::local_time();
#pragma omp parallel for
  for (size_type i=0; i<n2; ++i) {
#ifdef _OPENMP
    Nthreads = omp_get_num_threads();
#endif
#pragma omp critical
    {
      q1 = m1.row (i1++);
      q2 = m1.row (i1++);
      v1 = m2.row (i2++);
      v2 = m2.row (i2++);
    }
    PathValidationReportPtr_t report1;
    PathValidationReportPtr_t report2;
    PathValidationReportPtr_t report3;
    PathPtr_t path (sm->steer (q1, orders, v1, q2, orders, v2));
    PathPtr_t validPart;
    if (configValidation->validate (q1, collisionReport)) {
      bool res1 (discretized->validate (path, false, validPart, report1));
      bool res2 (progressive->validate  (path, false, validPart, report2));
      // bool res3 (dichotomy->validate (path, false, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK (!res2);
        // BOOST_CHECK (!res3);
        if (res2) {
          hppDout (error, "Progressive failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }
        /*if (res3) {
          hppDout (error, "Dichotomy failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
        }*/
      }
#pragma omp critical
      if (res1) {
        BOOST_CHECK (res2);
        if (!res2) {
          hppDout (info, "Progressive found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report2);
        }
        /*if (!res3) {
          hppDout (info, "Dichotomy found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report3);
        }*/
      }
    }
    if (configValidation->validate (q2, collisionReport)) {
      bool res1 (discretized->validate (path, true, validPart, report1));
      bool res2 (progressive->validate  (path, true, validPart, report2));
      // bool res3 (dichotomy->validate (path, true, validPart, report3));

#pragma omp critical
      if (!res1) {
        BOOST_CHECK (!res2);
        // BOOST_CHECK (!res3);
        if (res2) {
          hppDout (error, "Progressive failed to detect collision for q1="
                   << q1.transpose () << ", q2=" << q2.transpose ());
          hppDout (error, *report1);
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
          hppDout (info, "Progressive found a collision where discretized did "
		     "not for q1 = " << q1.transpose () << ", q2 = "
		   << q2.transpose ());
          hppDout (info, *report2);
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
  BOOST_MESSAGE ("Total time (nthreads " << Nthreads << "): " << (t1-t0).total_milliseconds() << "ms");
  // delete problem
}

BOOST_AUTO_TEST_CASE (continuous_validation_spline)
{
  test_spline_steering_method<hpp::core::steeringMethod::Spline<hpp::core::path::BernsteinBasis, 3> >();
}
#endif

BOOST_AUTO_TEST_SUITE_END()
