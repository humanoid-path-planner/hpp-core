// Copyright (c) 2016, Joseph Mirabel
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

#define BOOST_TEST_MODULE paths
#include <boost/test/included/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>

#include <boost/shared_ptr.hpp>
// Boost version 1.54
// Cannot include boost CPU timers
// #include <boost/timer/timer.hpp>
// because the original timers are already included by
// the unit test framework
// #include <boost/timer.hh>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/path.hh>
#include <hpp/core/straight-path.hh>

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;

using namespace hpp::core;

hpp::model::ObjectFactory objectFactory;

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("test");

  const std::string& name = robot->name ();
  fcl::Transform3f mat; mat.setIdentity ();
  JointPtr_t joint;
  std::string jointName = name + "_x";
  // Translation along x
  fcl::Matrix3f permutation;
  joint = objectFactory.createJointTranslation (mat);
  joint->name (jointName);

  joint->isBounded (0, 1);
  joint->lowerBound (0, -4);
  joint->upperBound (0, +4);

  robot->rootJoint (joint);
  return robot;
}

typedef std::pair<value_type, value_type> Pair_t;

std::ostream& operator<< (std::ostream& os, const Pair_t& p) {
  os << "Pair " << p.first << ", " << p.second;
  return os;
}

void printAt(const PathPtr_t& p, ConfigurationOut_t q, value_type t)
{
  (*p)(q, t);
  std::cout << t << ":\t" << q.transpose() << std::endl;
}
void checkAt(const PathPtr_t orig, value_type to,
             const PathPtr_t extr, value_type te) {
  Configuration_t q1 (orig->outputSize()),
                  q2 (orig->outputSize());
  (*orig)(q1, to);
  (*extr)(q2, te);
  BOOST_CHECK_MESSAGE(q2.isApprox(q1),
      "Extracted path is wrong."
      "\nExpected: " << q1.transpose() <<
      "\nGot     : " << q2.transpose()
      );
}

BOOST_AUTO_TEST_CASE (extracted)
{
  DevicePtr_t dev = createRobot ();
  BOOST_REQUIRE (dev);
  Problem problem (dev);

  Configuration_t q1 (dev->configSize()); q1 << 0;
  Configuration_t q2 (dev->configSize()); q2 << 1;
  PathPtr_t p1 = (*problem.steeringMethod()) (q1, q2), p2;

  p2 = p1->extract (Pair_t (0.5, 1));
  checkAt (p1, 0.5, p2, 0.0);
  checkAt (p1, 1.0, p2, 0.5);

  p2 = p1->extract (Pair_t (1, 0.5));
  checkAt (p1, 1.0, p2, 0.0);
  checkAt (p1, 0.5, p2, 0.5);

  p2 = p2->extract (Pair_t (0, 0.25));
  checkAt (p1, 1.0, p2, 0.0);
  checkAt (p1, .75, p2, .25);

  p2 = p1->extract (Pair_t (1, 0.5));
  p2 = p2->extract (Pair_t (0.25, 0));
  checkAt (p1, .75, p2, 0.0);
  checkAt (p1, 1.0, p2, .25);
}
