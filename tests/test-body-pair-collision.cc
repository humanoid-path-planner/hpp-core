//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE body_pair_collision

#include <limits>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/model/device.hh>

#include <hpp/pinocchio/joint.hh>
# include <hpp/pinocchio/device.hh>

#include "../src/continuous-collision-checking/dichotomy/body-pair-collision.hh"
#include <boost/test/included/unit_test.hpp>
#include "../tests/utils.hh"
#include <pinocchio/multibody/joint/joint-variant.hpp>




using std::numeric_limits;
using hpp::pinocchio::BodyPtr_t;
using hpp::pinocchio::CollisionObject;
using hpp::pinocchio::CollisionObjectPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::JointConstPtr_t;
using hpp::pinocchio::ObjectVector_t;
using hpp::core::continuousCollisionChecking::dichotomy::BodyPairCollision;
using hpp::core::continuousCollisionChecking::dichotomy::BodyPairCollisionPtr_t;

using namespace hpp::core;
using namespace hpp::pinocchio;

using ::se3::JointModelRX;
using ::se3::JointModelPX;
using ::se3::JointModelPY;
using ::se3::JointModelPZ;
using ::se3::JointModelRUBX;
using ::se3::JointModelFreeFlyer;
using ::se3::JointModelSpherical;
using ::se3::JointIndex;

void display(const se3::Model& model,
    const std::vector<se3::JointIndex>& joints) {
  std::vector <se3::JointIndex>::const_iterator it;
  for (it = joints.begin (); it != joints.end (); ++it) {
    if (*it > 0) std::cout << model.names[*it];
    else         std::cout << "None";
    std::cout << ", ";
  }
  std::cout << std::endl;
}


BOOST_AUTO_TEST_SUITE( test_hpp_core )

DevicePtr_t createRobot (){

  DevicePtr_t robot = Device::create ("test");
  const std::string& name = robot->name ();
  ModelPtr_t m = ModelPtr_t(new ::se3::Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new ::se3::GeometryModel());
  robot->model(m);
  robot->geomModel(gm);
  Transform3f mat; mat.setIdentity ();

  JointModelPX::TangentVector_t max_effort = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::TangentVector_t max_velocity = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::ConfigVector_t lower_position = JointModelPX::ConfigVector_t::Constant(-4.0);
  JointModelPX::ConfigVector_t upper_position = JointModelPX::ConfigVector_t::Constant(4.0);

  JointIndex idJoint = robot->model().addJoint(0,JointModelPX(), mat,name + "_x",max_effort,max_velocity,lower_position,upper_position);
  idJoint = robot->model().addJoint(idJoint,JointModelPY(), mat,name + "_y",max_effort,max_velocity,lower_position,upper_position);
  idJoint = robot->model().addJoint(idJoint,JointModelPZ(), mat,name + "_z",max_effort,max_velocity,lower_position,upper_position);

  JointIndex waist = robot->model().addJoint(idJoint,JointModelSpherical(), mat,name + "_SO3");

  JointModelRX::TangentVector_t max_effortRot = JointModelRX::TangentVector_t::Constant(JointModelRX::NV,std::numeric_limits<double>::max());
  JointModelRX::TangentVector_t max_velocityRot = JointModelRX::TangentVector_t::Constant(JointModelRX::NV,std::numeric_limits<double>::max());
  JointModelRX::ConfigVector_t lower_positionRot = JointModelRX::ConfigVector_t::Constant(-M_PI);
  JointModelRX::ConfigVector_t upper_positionRot = JointModelRX::ConfigVector_t::Constant(M_PI);

  Transform3f pos;
  fcl::Matrix3f orient;
  // Right leg joint 0
  orient (0,0) = 0; orient (0,1) = 0; orient (0,2) = -1;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) =  0;
  orient (2,0) = 1; orient (2,1) = 0; orient (2,2) =  0;
  pos.rotation (orient);
  pos.translation ( fcl::Vec3f(0, -0.08, 0));

  JointIndex idLeg = robot->model().addJoint(waist,JointModelRX(), pos,"RLEG_0",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY0");

  // Right leg joint 1
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, 0));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"RLEG_1",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY1");

  // Right leg joint 2
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, 0));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"RLEG_2",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY2");

  // Right leg joint 3: knee
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.35));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"RLEG_3",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY3");

  // Right leg joint 4: ankle
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.70));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"RLEG_4",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY4");

  // Right leg joint 5: ankle
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.70));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"RLEG_5",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"RLEG_BODY5");

  // left leg
  // left leg joint 0
  orient (0,0) = 0; orient (0,1) = 0; orient (0,2) = -1;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) =  0;
  orient (2,0) = 1; orient (2,1) = 0; orient (2,2) =  0;
  pos.rotation (orient);
  pos.translation ( fcl::Vec3f(0, -0.08, 0));

  idLeg = robot->model().addJoint(waist,JointModelRX(), pos,"LLEG_0",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY0");

  // left leg joint 1
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, 0));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"LLEG_1",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY1");

  // left leg joint 2
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, 0));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"LLEG_2",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY2");

  // left leg joint 3: knee
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.35));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"LLEG_3",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY3");

  // left leg joint 4: ankle
  orient (0,0) = 0; orient (0,1) = -1; orient (0,2) = 0;
  orient (1,0) = 1; orient (1,1) =  0; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) =  0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.70));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"LLEG_4",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY4");

  // left leg joint 5: ankle
  orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
  orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
  orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
  pos.rotation (orient);
  pos.translation (fcl::Vec3f (0, -0.08, -0.70));

  idLeg = robot->model().addJoint(idLeg,JointModelRX(), pos,"LLEG_5",max_effortRot,max_velocityRot,lower_positionRot,upper_positionRot);
  robot->model().appendBodyToJoint(idLeg,::se3::Inertia::Identity(),::se3::SE3::Identity(),"LLEG_BODY5");


  robot->createData();
  robot->createGeomData();
  return robot;
}

/*
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
  joint->lowerBound (0, -numeric_limits<double>::infinity());
  joint->upperBound (0, +numeric_limits<double>::infinity());
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
  joint->lowerBound (0, -numeric_limits<double>::infinity());
  joint->upperBound (0, +numeric_limits<double>::infinity());
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
  joint->lowerBound (0, -numeric_limits<double>::infinity());
  joint->upperBound (0, +numeric_limits<double>::infinity());
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
}*/



BOOST_AUTO_TEST_CASE (body_pair_collision_1)
{
  DevicePtr_t robot = createRobot();
  const se3::Model& model = robot->model();
  JointPtr_t joint_a = robot->getJointByBodyName ("LLEG_BODY5");
  JointPtr_t joint_b = robot->getJointByBodyName ("RLEG_BODY5");

  BodyPairCollisionPtr_t bpc = BodyPairCollision::create
    (joint_a, joint_b, 0.001);
  display(model, bpc->joints());

  ConstObjectStdVector_t obstacles;
  fcl::CollisionGeometryPtr_t box (new fcl::Box (.2, .4, .6));
  // FIXME this is a bit ugly.
  //robot->model().addFrame(se3::Frame("base_link",0,Transform3f (),se3::BODY));
  ::se3::GeomIndex idObj = robot->geomModel().addGeometryObject
    (se3::GeometryObject("obstacle", 0, 0, box, Transform3f (), ""));
  CollisionObjectPtr_t collObj (new hpp::pinocchio::CollisionObject(
        robot->geomModelPtr(), robot->geomDataPtr(), idObj));
  obstacles.push_back (collObj);
  bpc = BodyPairCollision::create (joint_a, obstacles, 0.001);
  display(model, bpc->joints());

  joint_a = robot->getJointByBodyName ("LLEG_BODY5");
  joint_b = robot->getJointByBodyName ("LLEG_BODY1");
  bpc = BodyPairCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->joints());

  joint_a = robot->getJointByBodyName ("LLEG_BODY1");
  joint_b = robot->getJointByBodyName ("LLEG_BODY5");
  bpc = BodyPairCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->joints());
}

BOOST_AUTO_TEST_SUITE_END()
