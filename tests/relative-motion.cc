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

#define BOOST_TEST_MODULE relativeMotion
#include <boost/test/included/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>

#include <boost/shared_ptr.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/relative-motion.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/numerical-constraint.hh>

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;

using hpp::constraints::RelativeTransformation;

using namespace hpp::core;

hpp::model::ObjectFactory objectFactory;

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("test");

  const std::string& name = robot->name ();
  fcl::Transform3f mat; mat.setIdentity ();
  JointPtr_t joint, root, parent;
  std::string jointName = name + "_x";
  // Translation along x
  root = objectFactory.createJointTranslation (mat);
  root->name (jointName);

  root->isBounded (0, 1);
  root->lowerBound (0, -4);
  root->upperBound (0, +4);

  robot->rootJoint (root);

  std::string bname = "joint_a";
  parent = root;
  for (int i = 0; i < 2; ++i) {
    joint = objectFactory.createJointTranslation (mat);
    std::stringstream ss; ss << bname << i;
    joint->name (ss.str());

    joint->isBounded (0, 1);
    joint->lowerBound (0, -4);
    joint->upperBound (0, +4);

    parent->addChildJoint (joint);
    parent = joint;
  }
  parent = root;
  bname = "joint_b";
  for (int i = 0; i < 3; ++i) {
    joint = objectFactory.createJointTranslation (mat);
    std::stringstream ss; ss << bname << i;
    joint->name (ss.str());

    joint->isBounded (0, 1);
    joint->lowerBound (0, -4);
    joint->upperBound (0, +4);

    parent->addChildJoint (joint);
    parent = joint;
  }
  return robot;
}

void lockJoint (ConfigProjectorPtr_t proj, DevicePtr_t dev, std::string name)
{
  JointPtr_t j = dev->getJointByName (name);
  Configuration_t q = dev->currentConfiguration();
  proj->add (
      LockedJoint::create(
        dev->getJointByName(name),
        q.segment(j->rankInConfiguration(), j->configSize()))
      );
}

BOOST_AUTO_TEST_CASE (relativeMotion)
{
  DevicePtr_t dev = createRobot ();
  BOOST_REQUIRE (dev);

  JointPtr_t ja1 = dev->getJointByName ("joint_a1"),
             jb2 = dev->getJointByName ("joint_b2");

  ConfigProjectorPtr_t proj = ConfigProjector::create (dev, "test", 1e-3, 10);
  ConstraintSetPtr_t constraints = ConstraintSet::create (dev, "test");
  constraints->addConstraint(proj);

  // Lock some joints
  lockJoint (proj, dev, "joint_b1");
  lockJoint (proj, dev, "joint_b2");
  lockJoint (proj, dev, "joint_a1");
  // root, a0, a1, b0, b1, b2
  // 0,    1,  2,  3,  4,  5

  RelativeMotion::matrix_type m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);
  BOOST_CHECK(m(2,3) == RelativeMotion::Parameterized); // lock a1
  BOOST_CHECK(m(4,5) == RelativeMotion::Parameterized); // lock b1
  BOOST_CHECK(m(5,6) == RelativeMotion::Parameterized); // lock b2
  BOOST_CHECK(m(4,6) == RelativeMotion::Parameterized); // lock b1+b2

  //std::cout << m << std::endl;

  // Add a relative transformation
  Configuration_t q = dev->neutralConfiguration();
  dev->currentConfiguration (q);
  dev->computeForwardKinematics ();
  Transform3f tf1 (ja1->currentTransformation ());
  Transform3f tf2 (jb2->currentTransformation ());
  proj->add (NumericalConstraint::create (
        RelativeTransformation::create ("", dev, ja1, jb2, tf1, tf2)));

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);
  BOOST_CHECK(m(3,6) == RelativeMotion::Constrained);   // lock rt
  BOOST_CHECK(m(2,6) == RelativeMotion::Parameterized); // lock a1 + rt
  BOOST_CHECK(m(4,3) == RelativeMotion::Parameterized); // lock b1+b2+rt
  BOOST_CHECK(m(4,2) == RelativeMotion::Parameterized); // lock b1+b2+rt+a1

  //std::cout << '\n' << m << std::endl;
}

