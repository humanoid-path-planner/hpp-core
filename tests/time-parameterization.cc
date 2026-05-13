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

#define BOOST_TEST_MODULE time_parameterization
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path-optimization/trapezoidal-time-parameterization.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/time-parameterization/piecewise-polynomial.hh>
#include <hpp/core/time-parameterization/polynomial.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/fwd.hpp>

using namespace hpp::core;
using namespace hpp::pinocchio;

DevicePtr_t createRobot() {
  std::string urdf(
      "<robot name='test'>"
      "<link name='link0'/>"
      "<joint name='joint0' type='prismatic'>"
      "<parent link='link0'/>"
      "<child  link='link1'/>"
      "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "<link name='link1'/>"
      "</robot>");

  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "", "anchor", urdf, "");
  return robot;
}

BOOST_AUTO_TEST_SUITE(polynomial)

BOOST_AUTO_TEST_CASE(linear) {
  vector_t a(vector_t::Ones(2));
  timeParameterization::Polynomial P(a);

  BOOST_CHECK_EQUAL(P.value(-1), 0);
  BOOST_CHECK_EQUAL(P.value(0), 1);
  BOOST_CHECK_EQUAL(P.value(1), 2);

  BOOST_CHECK_EQUAL(P.derivative(-1, 1), 1);
  BOOST_CHECK_EQUAL(P.derivative(0, 1), 1);
  BOOST_CHECK_EQUAL(P.derivative(1, 1), 1);

  BOOST_CHECK_EQUAL(P.derivativeBound(0, 1), 1);
  BOOST_CHECK_EQUAL(P.derivativeBound(-1, 1), 1);
  BOOST_CHECK_EQUAL(P.derivativeBound(-5, 5), 1);
}

BOOST_AUTO_TEST_CASE(cubic1) {
  vector_t a(vector_t::Ones(4));
  timeParameterization::Polynomial P(a);

  BOOST_CHECK_EQUAL(P.value(-1), 0);
  BOOST_CHECK_EQUAL(P.value(0), 1);
  BOOST_CHECK_EQUAL(P.value(1), 4);

  BOOST_CHECK_EQUAL(P.derivative(-2, 1), 9);
  BOOST_CHECK_EQUAL(P.derivative(-1, 1), 2);
  BOOST_CHECK_EQUAL(P.derivative(0, 1), 1);
  BOOST_CHECK_EQUAL(P.derivative(1, 1), 6);

  // x_m = - 1 / 3, P'(x_m) = 2/3
  BOOST_CHECK_EQUAL(P.derivativeBound(0, 1), 6);
  BOOST_CHECK_EQUAL(P.derivativeBound(-1, 0), 2);
  BOOST_CHECK_EQUAL(P.derivativeBound(-2, -1), 9);
}

BOOST_AUTO_TEST_CASE(cubic2) {
  vector_t a(vector_t::Ones(4));
  a[3] = -1;
  timeParameterization::Polynomial P(a);

  BOOST_CHECK_EQUAL(P.value(-1), 2);
  BOOST_CHECK_EQUAL(P.value(0), 1);
  BOOST_CHECK_EQUAL(P.value(1), 2);

  BOOST_CHECK_EQUAL(P.derivative(-1, 1), -4);
  BOOST_CHECK_EQUAL(P.derivative(0, 1), 1);
  BOOST_CHECK_EQUAL(P.derivative(1, 1), 0);

  // x_m = 1 / 3, P'(x_m) = 4 / 3
  BOOST_CHECK_EQUAL(P.derivativeBound(0, 1), 4. / 3);
  BOOST_CHECK_EQUAL(P.derivativeBound(-1, 0), 4);
  BOOST_CHECK_EQUAL(P.derivativeBound(1. / 3, 1), 4. / 3);
}

value_type integrate(const timeParameterization::Polynomial& P,
                     const value_type& L) {
  int N = 10000;

  value_type dt = L / N;
  value_type integral = 0;

  for (int i = 0; i < N; ++i) {
    integral += P.derivative(i * dt, 1) * dt;
  }
  return integral;
}

BOOST_AUTO_TEST_CASE(degree5integration) {
  value_type prec = 0.01;  // %
  vector_t a(6);
  {
    a << 0, 0, 0, 2.10515, -3.83792, 1.86585;
    timeParameterization::Polynomial P(a);
    value_type integral = integrate(P, 0.822769);
    BOOST_CHECK_CLOSE(integral, 0.117251, prec);
  }
  {
    a << 0, 0, 0, 1.13097, -1.10773, 0.289323;
    timeParameterization::Polynomial P(a);
    value_type integral = integrate(P, 1.53147);
    BOOST_CHECK_CLOSE(integral, 0.406237, prec);
  }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(path_optimizer)

BOOST_AUTO_TEST_CASE(trapezoidal_time_parameterization) {
  DevicePtr_t robot = createRobot();
  ProblemPtr_t problem = Problem::create(robot);

  pathOptimization::TrapezoidalTimeParameterizationPtr_t optimizer =
      pathOptimization::TrapezoidalTimeParameterization::create(problem);
  optimizer->maxVelocity = 1.;
  optimizer->maxAcceleration = 1.;
  optimizer->minimumDuration = 0.;

  Configuration_t q0(Configuration_t::Zero(robot->configSize()));
  Configuration_t q1(Configuration_t::Zero(robot->configSize()));
  q1[0] = 1.;

  PathVectorPtr_t path =
      PathVector::create(robot->configSize(), robot->numberDof());
  path->appendPath(StraightPath::create(robot, q0, q1, 1.));

  PathVectorPtr_t result = optimizer->optimize(path);
  BOOST_CHECK_CLOSE(result->length(), 2., 1e-10);

  Configuration_t q(robot->configSize());
  vector_t v(robot->numberDof());
  BOOST_CHECK(result->eval(q, 0.));
  BOOST_CHECK_SMALL((q - q0).norm(), 1e-12);
  BOOST_CHECK(result->eval(q, 1.));
  BOOST_CHECK_CLOSE(q[0], .5, 1e-10);
  BOOST_CHECK(result->eval(q, 2.));
  BOOST_CHECK_SMALL((q - q1).norm(), 1e-12);

  result->derivative(v, 0., 1);
  BOOST_CHECK_SMALL(v[0], 1e-12);
  result->derivative(v, 1., 1);
  BOOST_CHECK_CLOSE(v[0], 1., 1e-10);
  result->derivative(v, 2., 1);
  BOOST_CHECK_SMALL(v[0], 1e-12);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(piecewise_polynomial)

BOOST_AUTO_TEST_CASE(linear) {
  typedef timeParameterization::PiecewisePolynomial<1>
      LinearPiecewisePolynomial;
  LinearPiecewisePolynomial::ParameterMatrix_t M(2, 2);
  vector_t xs(3);
  // P(x) = x + 1 for x in [0, 1]
  // P(x) = 3 - x for x in [1, 2]
  M << 1, 3, 1, -1;
  xs << 0, 1, 2;

  LinearPiecewisePolynomial P(M, xs);

  BOOST_CHECK_THROW(P.value(-0.1), std::invalid_argument);
  BOOST_CHECK_THROW(P.value(2.1), std::invalid_argument);
  BOOST_CHECK_EQUAL(P.value(0), 1);
  BOOST_CHECK_EQUAL(P.value(1), 2);
  BOOST_CHECK_EQUAL(P.value(2), 1);
}

template <int Order>
value_type integrate(
    const timeParameterization::PiecewisePolynomial<Order>& P) {
  int N = 10000;

  std::vector<value_type> const& ts = P.breakpoints();
  value_type dt = (ts.back() - ts.front()) / N;
  value_type integral = 0;

  for (int i = 0; i < N; ++i) {
    integral += P.derivative(ts.front() + i * dt, 1) * dt;
  }
  return integral;
}

BOOST_AUTO_TEST_CASE(degree2integration) {
  typedef timeParameterization::PiecewisePolynomial<2> QuadPiecewisePolynomial;

  value_type prec = 0.01;  // %
  QuadPiecewisePolynomial::ParameterMatrix_t M(3, 3);
  vector_t ts(4);
  ts << 0.2, 1.4, 3, 3.1;
  {
    M << 0, 0.408, 2.424, 0.1, 0.3, 0.2, 0.2, 0.6, 1.5;
    QuadPiecewisePolynomial P(M, ts);

    BOOST_CHECK_CLOSE(P.value(0.3),
                      M(0, 0) + M(1, 0) * 0.3 + M(2, 0) * 0.3 * 0.3, 1e-13);
    BOOST_CHECK_CLOSE(P.derivative(0.3, 1), M(1, 0) + 2 * M(2, 0) * 0.3, 1e-13);
    P.polynomialsStartAtZero(true);
    BOOST_CHECK_CLOSE(P.value(0.3),
                      M(0, 0) + M(1, 0) * 0.1 + M(2, 0) * 0.1 * 0.1, 1e-13);
    BOOST_CHECK_CLOSE(P.derivative(0.3, 1), M(1, 0) + 2 * M(2, 0) * 0.1, 1e-13);

    for (int i : {1, 2, 3})
      BOOST_CHECK_CLOSE(P.value(ts[i] - 1e-12), P.value(ts[i]), 1e-8);

    value_type integral = integrate(P);
    BOOST_CHECK_CLOSE(integral, P.value(ts[3]) - P.value(ts[0]), prec);
  }
}

BOOST_AUTO_TEST_SUITE_END()
