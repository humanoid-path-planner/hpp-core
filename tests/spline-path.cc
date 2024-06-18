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

#define BOOST_TEST_MODULE spline_path
#include <../tests/util.hh>
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/fwd.hpp>

#define TOSTR(x) \
  static_cast<std::ostringstream&>((std::ostringstream() << x)).str()

using namespace hpp::core;
using namespace hpp::pinocchio;

DevicePtr_t createNDoFRobot(int ndof) {
  std::ostringstream oss;
  oss << "<robot name='test'>"
      << "<link name='link0'/>";
  for (int i = 0; i < ndof; ++i) {
    oss << "<joint name='joint" << i << "' type='prismatic'>"
        << "<parent link='link" << i << "'/>"
        << "<child  link='link" << i + 1 << "'/>"
        << "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
        << "</joint>"
        << "<link name='link" << i + 1 << "'/>";
  }
  oss << "</robot>";
  std::string urdf(oss.str());

  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "", "anchor", urdf, "");
  return robot;
}

DevicePtr_t createRobot() {
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidRomeo);
  robot->rootJoint()->lowerBound(0, -1);
  robot->rootJoint()->lowerBound(1, -1);
  robot->rootJoint()->lowerBound(2, -1);
  robot->rootJoint()->upperBound(0, 1);
  robot->rootJoint()->upperBound(1, 1);
  robot->rootJoint()->upperBound(2, 1);
  return robot;
}

DevicePtr_t createRobotArm() {
  DevicePtr_t robot = unittest::makeDevice(unittest::ManipulatorArm2);
  return robot;
}

typedef std::pair<value_type, value_type> Pair_t;

std::ostream& operator<<(std::ostream& os, const Pair_t& p) {
  os << "Pair " << p.first << ", " << p.second;
  return os;
}

void printAt(const PathPtr_t& p, ConfigurationOut_t q, value_type t) {
  p->eval(q, t);
  std::cout << t << ":\t" << q.transpose() << std::endl;
}
void checkAt(const PathPtr_t orig, value_type to, const PathPtr_t extr,
             value_type te) {
  Configuration_t q1(orig->outputSize()), q2(orig->outputSize());
  orig->eval(q1, to);
  extr->eval(q2, te);
  BOOST_CHECK_MESSAGE(q2.isApprox(q1),
                      "\nPath 1: " << q1.head<10>().transpose() << "\nPath 2: "
                                   << q2.head<10>().transpose());
}

template <int SplineType>
void compare_to_straight_path() {
  typedef path::Spline<SplineType, 1> path_t;
  typedef steeringMethod::Spline<SplineType, 1> SM_t;

  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);

  Configuration_t q1(::pinocchio::randomConfiguration(dev->model()));
  Configuration_t q2(::pinocchio::randomConfiguration(dev->model()));

  vector_t v(dev->numberDof());
  difference<RnxSOnLieGroupMap>(dev, q2, q1, v);

  // create StraightPath
  PathPtr_t sp = (*problem->steeringMethod())(q1, q2);
  // value_type length = sp->length();

  // Create linear spline
  typename SM_t::Ptr_t sm(SM_t::create(problem));
  PathPtr_t ls_abstract = (*sm)(q1, q2);
  typename path_t::Ptr_t ls = HPP_DYNAMIC_PTR_CAST(path_t, ls_abstract);

  /*
  typename path_t::Ptr_t ls = path_t::create (dev, interval_t(0, length),
  ConstraintSetPtr_t()); ls->base (q1); typename path_t::ParameterMatrix_t
  ls_param = ls->parameters(); ls_param.row(0).setZero(); ls_param.row(1) = v;
  ls->parameters(ls_param);
  */

  CONFIGURATION_VECTOR_IS_APPROX(dev, sp->initial(), ls->initial(), 1e-7);
  CONFIGURATION_VECTOR_IS_APPROX(dev, sp->end(), ls->end(), 1e-7);

  const size_type N = 10;
  const value_type step1 = sp->length() / N;
  const value_type step2 = ls->length() / N;

  // Check that straight path and linear spline return the same result.
  for (size_type i = 0; i < N; ++i)
    checkAt(sp, value_type(i) * step1, ls, value_type(i) * step2);

  // Check that the velocities are equals
  vector_t v1(dev->numberDof());
  vector_t v2(dev->numberDof());
  for (size_type i = 0; i < N; ++i) {
    sp->derivative(v1, value_type(i) * step1, 1);
    ls->derivative(v2, value_type(i) * step2, 1);
    BOOST_CHECK_SMALL((v2 * step2 - v1 * step1).squaredNorm(), 1e-12);
  }

  // Check integral value
  BOOST_CHECK_EQUAL(ls->squaredNormIntegral(2), 0);
  BOOST_CHECK_CLOSE(ls->squaredNormIntegral(1), v.squaredNorm() / sp->length(),
                    1e-12);

  vector_t derivative(dev->numberDof() * 2);
  ls->squaredNormIntegralDerivative(1, derivative);
  switch (SplineType) {
    case path::CanonicalPolynomeBasis:
      BOOST_CHECK(derivative.head(dev->numberDof()).isZero());
      EIGEN_VECTOR_IS_APPROX(derivative.tail(dev->numberDof()),
                             (2. / sp->length()) * v, 1e-6);
      break;
    case path::BernsteinBasis:
      EIGEN_VECTOR_IS_APPROX(derivative.head(dev->numberDof()),
                             (-2. / sp->length()) * v, 1e-6);
      EIGEN_VECTOR_IS_APPROX(derivative.tail(dev->numberDof()),
                             (2. / sp->length()) * v, 1e-6);
      break;
  }
}

template <int SplineType, int Degree>
void check_velocity_bounds() {
  typedef steeringMethod::Spline<SplineType, Degree> SM_t;

  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);

  Configuration_t q1(::pinocchio::randomConfiguration(dev->model()));
  Configuration_t q2(::pinocchio::randomConfiguration(dev->model()));
  std::vector<int> orders(1, 1);
  vector_t v1(vector_t::Random(dev->numberDof())),
      v2(vector_t::Random(dev->numberDof()));

  // Create spline
  typename SM_t::Ptr_t sm(SM_t::create(problem));
  PathPtr_t spline = sm->steer(q1, orders, v1, q2, orders, v2);

  vector_t vb1(-vector_t::Ones(dev->numberDof())), vb2 = vb1;
  value_type t0 = spline->timeRange().first, t1 = spline->timeRange().second;
  spline->velocityBound(vb1, t0, t1);

  BOOST_CHECK((vb1.array() >= 0).all());

  std::size_t N = 1000;
  value_type step = spline->length() / value_type(N);
  for (std::size_t i = 0; i < N; ++i) {
    vb2.setConstant(-1);
    spline->velocityBound(vb2, t0, t1);
    BOOST_CHECK((vb2.array() >= 0).all());
    BOOST_CHECK_MESSAGE(
        (vb2.array() <= vb1.array()).all(),
        "i=" << i << " Velocity bound should have decreased. Interval is ["
             << t0 << ", " << t1 << "]. Difference:\n"
             << (vb1 - vb2).transpose());
    vb1 = vb2;
    if (i % 2) {
      t0 += step;
    } else {
      t1 -= step;
    }
  }
}

template <int SplineType, int Degree, int order>
void check_steering_method() {
  typedef steeringMethod::Spline<SplineType, Degree> SM_t;
  std::vector<int> orders{1};
  if (order == 2) orders.push_back(2);

  // Use the manipulator arm and not Romeo since steering method does not give
  // correct values for vel/acc when the robot configuration contains a
  // freeflyer
  DevicePtr_t dev = createRobotArm();
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);

  // Create random configurations and velocities/accelerations
  Configuration_t q1(::pinocchio::randomConfiguration(dev->model()));
  Configuration_t q2(::pinocchio::randomConfiguration(dev->model()));
  matrix_t deriv1(matrix_t::Random(dev->numberDof(), order)),
      deriv2(matrix_t::Random(dev->numberDof(), order));
  double length = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  // Create spline
  typename SM_t::Ptr_t sm(SM_t::create(problem));
  PathPtr_t spline1 = sm->steer(q1, orders, deriv1, q2, orders, deriv2, length);

  // Check length
  double spline_length = spline1->length();
  BOOST_CHECK_MESSAGE(abs(spline_length - length) < 0.0001,
                      "Path does not have desired length: "
                          << spline_length << " instead of " << length);

  // Check configuration at start/end
  Configuration_t spline_q1 = spline1->initial();
  Configuration_t spline_q2 = spline1->end();
  EIGEN_VECTOR_IS_APPROX(q1, spline_q1, 1e-6);
  EIGEN_VECTOR_IS_APPROX(q2, spline_q2, 1e-6);

  // Check derivatives at start/end
  for (int i = 1; i <= order; i++) {
    vector_t spline_v1(vector_t::Random(dev->numberDof())),
        spline_v2 = spline_v1;
    spline1->derivative(spline_v1, 0, i);
    spline1->derivative(spline_v2, spline_length, i);
    EIGEN_VECTOR_IS_APPROX(deriv1.col(i - 1), spline_v1, 1e-6);
    EIGEN_VECTOR_IS_APPROX(deriv2.col(i - 1), spline_v2, 1e-6);
  }
}

BOOST_AUTO_TEST_CASE(spline_bernstein) {
  compare_to_straight_path<path::BernsteinBasis>();
}

void check_bernstein_polynomial_3rd(value_type p0, value_type p1, value_type p2,
                                    value_type p3) {
  // Degree 3
  typedef path::Spline<path::BernsteinBasis, 3> path_t;

  DevicePtr_t dev = createNDoFRobot(1);
  vector_t q0(1);
  q0 << 0.0;
  path_t::ParameterMatrix_t m(4, 1);
  m << p0, p1, p2, p3;
  bool ok;

  ConstraintSetPtr_t constraint = ConstraintSet::create(dev, "empty");
  path_t::Ptr_t path = path_t::create(dev, interval_t(0, 1), constraint);
  path->base(q0);
  path->parameters(m);

  // evaluation
  BOOST_CHECK_CLOSE(path->eval(0.0, ok)[0], m(0, 0), 1e-8);
  BOOST_CHECK_CLOSE(path->eval(0.5, ok)[0],
                    0.125 * (m(0, 0) + 3 * m(1, 0) + 3 * m(2, 0) + m(3, 0)),
                    1e-8);
  BOOST_CHECK_CLOSE(path->eval(1.0, ok)[0], m(3, 0), 1e-8);

  // derivative
  vector_t der(1);
  path->derivative(der, 0.0, ok);
  BOOST_CHECK_CLOSE(der[0], 3 * (-m(0, 0) + m(1, 0)), 1e-8);
  path->derivative(der, 0.5, ok);
  BOOST_CHECK_CLOSE(der[0], 3 * (-m(0, 0) - m(1, 0) + m(2, 0) + m(3, 0)) / 4,
                    1e-8);
  path->derivative(der, 1.0, ok);
  BOOST_CHECK_CLOSE(der[0], 3 * (-m(2, 0) + m(3, 0)), 1e-8);

  // velocity bound
  vector_t vb(1);
  vector_t splineVbounds(4);
  path->velocityBound(vb, 0.0, 1.0);
  splineVbounds << 3, 3, 3, 3;
  BOOST_CHECK_CLOSE(vb[0], (m.cwiseAbs().transpose() * splineVbounds)[0], 1e-8);

  path->velocityBound(vb, 0.0, 0.5);
  splineVbounds << 3, 3, 1, 0.75;
  BOOST_CHECK_CLOSE(vb[0], (m.cwiseAbs().transpose() * splineVbounds)[0], 1e-8);

  path->velocityBound(vb, 0.5, 1.0);
  splineVbounds << 0.75, 1, 3, 3;
  BOOST_CHECK_CLOSE(vb[0], (m.cwiseAbs().transpose() * splineVbounds)[0], 1e-8);

  path->velocityBound(vb, 0.5, 0.6);
  splineVbounds << 0.75, 0.96, 0.75, 1.08;
  BOOST_CHECK_CLOSE(vb[0], (m.cwiseAbs().transpose() * splineVbounds)[0], 1e-8);
}

BOOST_AUTO_TEST_CASE(bernstein_polynomial) {
  check_bernstein_polynomial_3rd(1.0, 0.0, 0.0, 0.0);
  check_bernstein_polynomial_3rd(0.0, 1.0, 0.0, 0.0);
  check_bernstein_polynomial_3rd(0.0, 0.0, 1.0, 0.0);
  check_bernstein_polynomial_3rd(0.0, 0.0, 0.0, 1.0);

  check_bernstein_polynomial_3rd(1.0, 0.3, 0.4, -0.5);
}

BOOST_AUTO_TEST_CASE(spline_bernstein_velocity) {
  check_velocity_bounds<path::BernsteinBasis, 3>();
}

BOOST_AUTO_TEST_CASE(steering_method) {
  check_steering_method<path::BernsteinBasis, 3, 1>();
  check_steering_method<path::BernsteinBasis, 5, 1>();
  check_steering_method<path::BernsteinBasis, 5, 2>();
}

BOOST_AUTO_TEST_CASE(hermite_path) {
  DevicePtr_t dev = createRobotArm();
  ConstraintSetPtr_t constraint =
      hpp::core::ConstraintSet::create(dev, "empty");

  vector_t q1(vector_t::Zero(dev->configSize()));
  vector_t q2(vector_t::Ones(dev->configSize()));

  path::HermitePtr_t path =
      hpp::core::path::Hermite::create(dev, q1, q2, constraint);
  bool ok;
  vector_t evaluated = path->eval(path->length(), ok);
  BOOST_CHECK(ok);
  CONFIGURATION_VECTOR_IS_APPROX(dev, evaluated, path->end(), 1e-12);
}
