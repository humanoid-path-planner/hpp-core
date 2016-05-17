// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE ConfigProjector 
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/config-projector.hh>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/constraints/generic-transformation.hh>

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::model::BodyPtr_t;

using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::matrix3_t;
using hpp::constraints::vector3_t;

using namespace hpp::core;

hpp::model::ObjectFactory objectFactory;

JointPtr_t createFreeflyerJoint (DevicePtr_t robot)
{
  const std::string& name = robot->name ();
  fcl::Transform3f mat; mat.setIdentity ();
  JointPtr_t joint, parent;
  const fcl::Vec3f T = mat.getTranslation ();
  std::string jointName = name + "_x";
  // Translation along x
  fcl::Matrix3f permutation;
  joint = objectFactory.createJointTranslation (mat);
  joint->name (jointName);
  joint->isBounded (0, 1);
  joint->lowerBound (0, -4);
  joint->upperBound (0, +4);
  robot->rootJoint (joint);
  parent = joint;

  // Translation along y
  permutation (0,0) = 0; permutation (0,1) = -1; permutation (0,2) = 0;
  permutation (1,0) = 1; permutation (1,1) =  0; permutation (1,2) = 0;
  permutation (2,0) = 0; permutation (2,1) =  0; permutation (2,2) = 1;
  fcl::Transform3f pos;
  pos.setRotation (permutation * mat.getRotation ());
  pos.setTranslation (T);
  joint = objectFactory.createJointTranslation (pos);
  jointName = name + "_y";
  joint->name (jointName);
  joint->isBounded (0, 1);
  joint->lowerBound (0, -4);
  joint->upperBound (0, +4);
  parent->addChildJoint (joint);
  parent = joint;

  // Translation along z
  permutation (0,0) = 0; permutation (0,1) = 0; permutation (0,2) = -1;
  permutation (1,0) = 0; permutation (1,1) = 1; permutation (1,2) =  0;
  permutation (2,0) = 1; permutation (2,1) = 0; permutation (2,2) =  0;
  pos.setRotation (permutation * mat.getRotation ());
  pos.setTranslation (T);
  joint = objectFactory.createJointTranslation (pos);
  jointName = name + "_z";
  joint->name (jointName);
  joint->isBounded (0, 1);
  joint->lowerBound (0, -4);
  joint->upperBound (0, +4);
  parent->addChildJoint (joint);
  parent = joint;
  // joint SO3
  joint = objectFactory.createJointSO3 (mat);
  jointName = name + "_SO3";
  joint->name (jointName);
  parent->addChildJoint (joint);
  return joint;
}

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("test");
  JointPtr_t waist = createFreeflyerJoint (robot);
  JointPtr_t parent = waist;
  BodyPtr_t body;
  fcl::Transform3f pos;
  fcl::Matrix3f orient;
  // Right leg joint 0
  orient (0,0) = 0; orient (0,1) = 0; orient (0,2) = -1;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) =  0;
  orient (2,0) = 1; orient (2,1) = 0; orient (2,2) =  0;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  JointPtr_t joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_0");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY0");
  joint->setLinkedBody (body);
  parent = joint;
  // Right leg joint 1
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_1");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY1");
  joint->setLinkedBody (body);
  parent = joint;
  // Right leg joint 2
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_2");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY2");
  joint->setLinkedBody (body);
  parent = joint;
  // Right leg joint 3: knee
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.35));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_3");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY3");
  joint->setLinkedBody (body);
  parent = joint;
  // Right leg joint 4: ankle
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.70));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_4");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY4");
  joint->setLinkedBody (body);
  parent = joint;
  // Right leg joint 5: ankle
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.70));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("RLEG_5");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("RLEG_BODY5");
  joint->setLinkedBody (body);

  // Left leg
  parent = waist;
  // Left leg joint 0
  orient (0,0) = 0; orient (0,1) = 0; orient (0,2) = -1;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) =  0;
  orient (2,0) = 1; orient (2,1) = 0; orient (2,2) =  0;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_0");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY0");
  joint->setLinkedBody (body);
  parent = joint;
  // Left leg joint 1
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_1");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY1");
  joint->setLinkedBody (body);
  parent = joint;
  // Left leg joint 2
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, 0));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_2");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY2");
  joint->setLinkedBody (body);
  parent = joint;
  // Left leg joint 3: knee
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.35));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_3");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY3");
  joint->setLinkedBody (body);
  parent = joint;
  // Left leg joint 4: ankle
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.70));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_4");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY4");
  joint->setLinkedBody (body);
  parent = joint;
  // Left leg joint 5: ankle
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.setRotation (orient);
  pos.setTranslation (fcl::Vec3f (0, -0.08, -0.70));
  joint = objectFactory.createBoundedJointRotation (pos);
  joint->name ("LLEG_5");
  parent->addChildJoint (joint);
  body = objectFactory.createBody ();
  body->name ("LLEG_BODY5");
  joint->setLinkedBody (body);

  return robot;
}

BOOST_AUTO_TEST_SUITE (config_projector)

BOOST_AUTO_TEST_CASE (ref_zero)
{
  DevicePtr_t dev = createRobot ();
  JointPtr_t xyz = dev->getJointByName ("test_z");
  matrix3_t rot; rot.setIdentity ();
  vector3_t zero; zero.setZero();
  ComparisonType::VectorOfTypes types (3, ComparisonType::Superior);
  BOOST_REQUIRE (dev);
  PositionPtr_t position =
    Position::create ("Position", dev, xyz, zero);

  types[1] = ComparisonType::Inferior;
  ComparisonTypesPtr_t ineq = ComparisonTypes::create (types);
  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);
  projector->add (NumericalConstraint::create (position, ineq));
  Configuration_t cfg(dev->configSize ());
  cfg.setZero ();
  cfg [3] = 1; // Normalize quaternion

  vector_t invert(3);
  invert[0] =  1;
  invert[1] = -1;
  invert[2] =  1;
  cfg.segment (0,3) = invert;
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ((cfg.segment (0, 3) - invert).isZero (), "Configuration should not be modified.");

  cfg.segment (0,3) = vector_t::Ones (3);
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ((cfg.segment (0,1) - vector_t::Ones(1)).isZero (), "Dof 0 should not have been modified.");
  BOOST_CHECK_MESSAGE ( cfg (1) < 0                                     , "Dof 1 should have been modified.");
  BOOST_CHECK_MESSAGE ((cfg.segment (2,1) - vector_t::Ones(1)).isZero (), "Dof 2 should not have been modified.");
}

BOOST_AUTO_TEST_CASE (ref_not_zero)
{
  DevicePtr_t dev = createRobot ();
  JointPtr_t xyz = dev->getJointByName ("test_z");
  matrix3_t rot; rot.setIdentity ();
  vector3_t zero; zero.setZero();
  vector_t ref(3);
  ComparisonType::VectorOfTypes types (3, ComparisonType::Superior);
  BOOST_REQUIRE (dev);
  PositionPtr_t position =
    Position::create ("Position", dev, xyz, zero, vector3_t (1,1,1));

  ref[0] = 0; ref[1] = 0; ref[2] = 0;
  types[1] = ComparisonType::Inferior;
  ComparisonTypesPtr_t ineq = ComparisonTypes::create (types);
  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);
  projector->add (NumericalConstraint::create (position, ineq));
  Configuration_t cfg(dev->configSize ());
  cfg.setZero ();
  cfg [3] = 1; // Normalize quaternion

  ref[0] = 2; 
  ref[1] = 0; 
  ref[2] = 2; 
  cfg.segment (0,3) = ref;
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ((cfg.segment (0, 3) - ref).isZero (), "Configuration should not be modified.");

  ref[0] = 0; 
  ref[1] = 0; 
  ref[2] = 0; 
  cfg.segment (0,3) = ref;
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ( cfg (0) > 1                                     , "Dof 0 should have been modified.");
  BOOST_CHECK_MESSAGE ((cfg.segment (1,1) - ref.segment (1,1)).isZero (), "Dof 1 should not have been modified.");
  BOOST_CHECK_MESSAGE ( cfg (2) > 1                                     , "Dof 2 should have been modified.");
}

BOOST_AUTO_TEST_SUITE_END()
