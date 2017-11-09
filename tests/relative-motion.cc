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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/relative-motion.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/numerical-constraint.hh>
#include <hpp/core/explicit-relative-transformation.hh>
#include <pinocchio/multibody/joint/joint-variant.hpp>
#include <pinocchio/multibody/geometry.hpp>

using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointPtr_t;

using hpp::constraints::RelativeTransformation;

using namespace hpp::core;
using namespace hpp::pinocchio;

using ::se3::JointModelFreeFlyer;
using ::se3::JointModelPX;
using ::se3::JointModelPY;
using ::se3::JointModelPZ;
using ::se3::JointIndex;

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

  DevicePtr_t robot = Device::create ("test");
  const std::string& name = robot->name ();
  ModelPtr_t m = ModelPtr_t(new ::se3::Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new ::se3::GeometryModel());
  robot->model(m);
  robot->geomModel(gm);
  Transform3f mat; mat.setIdentity ();
  std::string jointName = name + "_x";

  JointModelPX::TangentVector_t max_effort = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::TangentVector_t max_velocity = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::ConfigVector_t lower_position = JointModelPY::ConfigVector_t::Constant(-4);
  JointModelPX::ConfigVector_t upper_position = JointModelPY::ConfigVector_t::Constant(4);

  JointIndex idJoint = robot->model().addJoint(0,JointModelPX(), mat,jointName,max_effort,max_velocity,lower_position,upper_position);

  std::string bname = "joint_a";
  for (int i = 0; i < 2; ++i) {
    idJoint = robot->model().addJoint(idJoint,JointModelPX(), mat,bname+TOSTR(i),max_effort,max_velocity,lower_position,upper_position);
  }
  bname = "joint_b";
  idJoint = 1;
  for (int i = 0; i < 2; ++i) {
    idJoint = robot->model().addJoint(idJoint,JointModelPX(), mat,bname+TOSTR(i),max_effort,max_velocity,lower_position,upper_position);
  }
  int i = 2;
  idJoint = robot->model().addJoint(idJoint,JointModelFreeFlyer(), mat,bname+TOSTR(i));

  robot->createData();
  robot->createGeomData();
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
    return RelativeMotion::idx(dev->getJointByName (jointname));
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
  BOOST_CHECK(jointid("universe") == 0);
  BOOST_CHECK(jointid("test_x")   == 1);
  BOOST_CHECK(jointid("joint_a0") == 2);
  BOOST_CHECK(jointid("joint_a1") == 3);
  BOOST_CHECK(jointid("joint_b0") == 4);
  BOOST_CHECK(jointid("joint_b1") == 5);
  BOOST_CHECK(jointid("joint_b2") == 6);

  // Lock some joints
  lockJoint (proj, dev, "joint_b1");

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK(m(jointid("joint_b0"),jointid("joint_b1")) == RelativeMotion::Parameterized); // lock b1

  if (verbose) std::cout << '\n' << m << std::endl;

  // Lock some joints
  lockJoint (proj, dev, "joint_b2");
  lockJoint (proj, dev, "joint_a1");

  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);

  BOOST_CHECK(m(jointid("joint_a0"),jointid("joint_a1")) == RelativeMotion::Parameterized); // lock a1
  BOOST_CHECK(m(jointid("joint_b0"),jointid("joint_b1")) == RelativeMotion::Parameterized); // lock b1
  BOOST_CHECK(m(jointid("joint_b1"),jointid("joint_b2")) == RelativeMotion::Parameterized); // lock b2

  BOOST_CHECK(m(jointid("joint_b0"),jointid("joint_b2")) == RelativeMotion::Parameterized); // lock b1+b2

  if (verbose) std::cout << '\n' << m << std::endl;

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

  if (verbose) std::cout << '\n' << m << std::endl;

  BOOST_CHECK(m(jointid("joint_a1"),jointid("joint_b2")) == RelativeMotion::Constrained);   // lock rt
  BOOST_CHECK(m(jointid("joint_a0"),jointid("joint_b2")) == RelativeMotion::Parameterized); // lock a1 + rt
  BOOST_CHECK(m(jointid("joint_b0"),jointid("joint_a1")) == RelativeMotion::Parameterized); // lock b1+b2+rt
  BOOST_CHECK(m(jointid("joint_b0"),jointid("joint_a0")) == RelativeMotion::Parameterized); // lock b1+b2+rt+a1

  if (verbose) std::cout << '\n' << m << std::endl;

  proj = ConfigProjector::create (dev, "test", 1e-3, 10);
  constraints = ConstraintSet::create (dev, "test");
  constraints->addConstraint(proj);
  proj->add (ExplicitRelativeTransformation::create ("", dev, ja1, jb2, tf1, tf2)->createNumericalConstraint());
  m = RelativeMotion::matrix(dev);
  RelativeMotion::fromConstraint (m, dev, constraints);
  BOOST_CHECK(m(jointid("joint_a1"),jointid("joint_b2")) == RelativeMotion::Constrained);   // lock ert

  if (verbose) std::cout << '\n' << m << std::endl;
}
