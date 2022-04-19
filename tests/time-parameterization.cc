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
#include <hpp/core/time-parameterization/polynomial.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <pinocchio/fwd.hpp>

using namespace hpp::core;
using namespace hpp::pinocchio;

DevicePtr_t createRobot() {
  DevicePtr_t robot = unittest::makeDevice(unittest::ManipulatorArm2);
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
