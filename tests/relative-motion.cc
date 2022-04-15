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

#define BOOST_TEST_MODULE relativeMotion
#include <pinocchio/fwd.hpp>
#include <boost/test/included/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>

#include <boost/shared_ptr.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/relative-motion.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/comparison-types.hh>
#include <hpp/constraints/explicit/relative-pose.hh>

using hpp::constraints::RelativeTransformation;
using hpp::constraints::Implicit;
using hpp::constraints::explicit_::RelativePose;
using hpp::constraints::Equality;
using hpp::constraints::EqualToZero;

using namespace hpp::core;
using namespace hpp::pinocchio;

bool verbose = true;

#define TOSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << x ) ).str()


/* Create a robot with the following kinematic chain. All joints are
   translations along x.

                               universe
                                  |Px
                               test_x
                             /Px       \Px
                       joint_a0       joint_b0
                           |Px            |Px
                       joint_a1       joint_b1
                                          |FF
                                      joint_b2

*/
DevicePtr_t createRobot ()
{
  std::string urdf ("<robot name='test'>"
      "<link name='base_link'/>"
      "<link name='link_test_x'/>"
      "<joint name='test_x' type='prismatic'>"
        "<parent link='base_link'/>"
        "<child  link='link_test_x'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"

      "<link name='link_a0'/>"
      "<link name='link_a1'/>"
      "<joint name='joint_a0' type='prismatic'>"
        "<parent link='link_test_x'/>"
        "<child  link='link_a0'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "<joint name='joint_a1' type='prismatic'>"
        "<parent link='link_a0'/>"
        "<child  link='link_a1'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"

      "<link name='link_b0'/>"
      "<link name='link_b1'/>"
      "<link name='link_b2'/>"
      "<joint name='joint_b0' type='prismatic'>"
        "<parent link='link_test_x'/>"
        "<child  link='link_b0'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "<joint name='joint_b1' type='prismatic'>"
        "<parent link='link_b0'/>"
        "<child  link='link_b1'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "<joint name='joint_b2' type='floating'>"
        "<parent link='link_b1'/>"
        "<child  link='link_b2'/>"
      "</joint>"

      "</robot>"
      );


  DevicePtr_t robot = Device::create ("test");
  urdf::loadModelFromString (robot, 0, "", "anchor", urdf, "");
  return robot;
}

void lockJoint (ConfigProjectorPtr_t proj, DevicePtr_t dev, std::string name)
{
  JointPtr_t j = dev->getJointByName (name);
  LiegroupSpacePtr_t space (j->configurationSpace ());
  LiegroupElement lge (space);
  Configuration_t q = dev->currentConfiguration();
  lge.vector () = q.segment(j->rankInConfiguration(), j->configSize());
  lge.check ();
  proj->add (
      LockedJoint::create(
        dev->getJointByName(name),
        lge)
      );
}

struct Jidx {
  DevicePtr_t dev;
  size_type operator() (const std::string& jointname)
  {
    return Joint::index(dev->getJointByName (jointname));
  }
};

BOOST_AUTO_TEST_CASE (relativeMotion)
{
  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE (dev);
  Jidx jointid;
  jointid.dev = dev;

  JointConstPtr_t ja1 = dev->getJointByName ("joint_a1"),
                  jb2 = dev->getJointByName ("joint_b2");

  RelativeMotion::matrix_type m;

  ConfigProjectorPtr_t proj = ConfigProjector::create (dev, "test", 1e-3, 10);
  ConstraintSetPtr_t constraints = ConstraintSet::create (dev, "test");
  constraints->addConstraint(proj);

  // root, x, a0, a1, b0, b1, b2
  // 0,    1,  2,  3,  4,  5,  6
  BOOST_CHECK_EQUAL(jointid("universe"), 0);
  BOOST_CHECK_EQUAL(jointid("test_x")  , 1);
  BOOST_CHECK_EQUAL(jointid("joint_a0"), 2);
  BOOST_CHECK_EQUAL(jointid("joint_a1"), 3);
  BOOST_CHECK_EQUAL(jointid("joint_b0"), 4);
  BOOST_CHECK_EQUAL(jointid("joint_b1"), 5);
  BOOST_CHECK_EQUAL(jointid("joint_b2"), 6);

  // Lock some joints
  lockJoint (proj, dev, "joint_b1");

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_b1")), RelativeMotion::Parameterized); // lock b1

  if (verbose) std::cout << '\n' << m << std::endl;

  // Lock some joints
  lockJoint (proj, dev, "joint_b2");
  lockJoint (proj, dev, "joint_a1");

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK_EQUAL(m(jointid("joint_a0"),jointid("joint_a1")), RelativeMotion::Parameterized); // lock a1
  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_b1")), RelativeMotion::Parameterized); // lock b1
  BOOST_CHECK_EQUAL(m(jointid("joint_b1"),jointid("joint_b2")), RelativeMotion::Parameterized); // lock b2

  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_b2")), RelativeMotion::Parameterized); // lock b1+b2

  if (verbose) std::cout << '\n' << m << std::endl;

  /// Add a relative transformation
  Configuration_t q = dev->neutralConfiguration();
  dev->currentConfiguration (q);
  dev->computeForwardKinematics ();
  Transform3f tf1 (ja1->currentTransformation ());
  Transform3f tf2 (jb2->currentTransformation ());
  // mask is not full, relative motion not fully constrained
  proj->add (Implicit::create
             (RelativeTransformation::create ("joint_a1 <->joint_b2 not full",
                                              dev, ja1, jb2, tf1, tf2),
              EqualToZero << EqualToZero << 3*Equality << EqualToZero,
              std::vector<bool> (6, false)));

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK_EQUAL(m(jointid("joint_a1"),jointid("joint_b2")), RelativeMotion::Unconstrained);
  BOOST_CHECK_EQUAL(m(jointid("joint_a0"),jointid("joint_b2")), RelativeMotion::Unconstrained);
  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_a1")), RelativeMotion::Unconstrained);
  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_a0")), RelativeMotion::Unconstrained);

  if (verbose) std::cout << '\n' << m << std::endl;
  // full mask for relative transformation
  proj->add (Implicit::create
             (RelativeTransformation::create ("joint_a1 <->joint_b2", dev, ja1,
                                              jb2, tf1, tf2),
              EqualToZero << EqualToZero << 3*Equality << EqualToZero));

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK_EQUAL(m(jointid("joint_a1"),jointid("joint_b2")), RelativeMotion::Parameterized);   // lock rt
  BOOST_CHECK_EQUAL(m(jointid("joint_a0"),jointid("joint_b2")), RelativeMotion::Parameterized); // lock a1 + rt
  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_a1")), RelativeMotion::Parameterized); // lock b1+b2+rt
  BOOST_CHECK_EQUAL(m(jointid("joint_b0"),jointid("joint_a0")), RelativeMotion::Parameterized); // lock b1+b2+rt+a1

  if (verbose) std::cout << '\n' << m << std::endl;

  proj = ConfigProjector::create (dev, "test", 1e-3, 10);
  constraints = ConstraintSet::create (dev, "test");
  constraints->addConstraint(proj);
  proj->add (RelativePose::create
             ("explicit joint_a1 <-> joint_b2", dev, ja1, jb2, tf1, tf2,
              2 * Equality << 3 * EqualToZero << Equality));
  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);
  BOOST_CHECK_EQUAL (m(jointid("joint_a1"),jointid("joint_b2")), RelativeMotion::Parameterized);   // lock ert

  if (verbose) std::cout << '\n' << m << std::endl;
}
