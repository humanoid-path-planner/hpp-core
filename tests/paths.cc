// Copyright (c) 2016, Joseph Mirabel
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

#define BOOST_TEST_MODULE paths
#include <boost/mpl/list.hpp>
#include <boost/test/included/unit_test.hpp>
#include <pinocchio/fwd.hpp>

// Boost version 1.54
// Cannot include boost CPU timers
// #include <boost/timer/timer.hpp>
// because the original timers are already included by
// the unit test framework
// #include <boost/timer.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/subchain-path.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#define TOSTR(x) \
  static_cast<std::ostringstream&>((std::ostringstream() << x)).str()

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

DevicePtr_t createRobot2() {
  std::ostringstream oss;
  oss << "<robot name='test'>"
      << "<link name='link0'/>";
  for (int i = 0; i < 10; ++i) {
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
                      "Extracted path is wrong."
                      "\nExpected: "
                          << q1.transpose()
                          << "\nGot     : " << q2.transpose());
}

BOOST_AUTO_TEST_CASE(extracted) {
  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);

  Configuration_t q1(dev->configSize());
  q1 << 0;
  Configuration_t q2(dev->configSize());
  q2 << 1;
  PathPtr_t p1 = (*problem->steeringMethod())(q1, q2), p2;

  p2 = p1->extract(Pair_t(0.5, 1));
  checkAt(p1, 0.5, p2, 0.0);
  checkAt(p1, 1.0, p2, 0.5);

  p2 = p1->extract(Pair_t(1, 0.5));
  checkAt(p1, 1.0, p2, 0.0);
  checkAt(p1, 0.5, p2, 0.5);

  p2 = p2->extract(Pair_t(0, 0.25));
  checkAt(p1, 1.0, p2, 0.0);
  checkAt(p1, .75, p2, .25);

  p2 = p1->extract(Pair_t(1, 0.5));
  p2 = p2->extract(Pair_t(0.25, 0));
  checkAt(p1, .75, p2, 0.0);
  checkAt(p1, 1.0, p2, .25);
}

BOOST_AUTO_TEST_CASE(subchain) {
  DevicePtr_t dev = createRobot2();  // 10 translations
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);

  segments_t intervals;
  intervals.push_back(segment_t(0, 3));
  intervals.push_back(segment_t(6, 3));

  Configuration_t q1(Configuration_t::Zero(dev->configSize()));
  Configuration_t q2(Configuration_t::Ones(dev->configSize()));
  q2.tail<5>().setConstant(-1);

  PathPtr_t p1 = (*problem->steeringMethod())(q1, q2), p2;
  p2 = SubchainPath::create(p1, intervals, intervals);

  BOOST_CHECK(p2->outputSize() == 6);
  Configuration_t q(p2->outputSize());
  p2->eval(q, 0);
  BOOST_CHECK(q.head<3>().isZero());
  BOOST_CHECK(q.tail<3>().isZero());

  p2->eval(q, p1->length());
  BOOST_CHECK(q.head<3>().isApprox(Configuration_t::Ones(3)));
  BOOST_CHECK(q.tail<3>().isApprox(-Configuration_t::Ones(3)));

  p2->eval(q, p1->length() * 0.5);
  BOOST_CHECK(q.head<3>().isApprox(Configuration_t::Ones(3) * 0.5));
  BOOST_CHECK(q.tail<3>().isApprox(-Configuration_t::Ones(3) * 0.5));
}
