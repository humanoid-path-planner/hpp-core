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
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/simple-device.hh>

#include "../src/continuous-collision-checking/body-pair-collision.hh"
#include <boost/test/included/unit_test.hpp>

using std::numeric_limits;
using namespace hpp::core;
using namespace hpp::pinocchio;

using hpp::core::continuousCollisionChecking::BodyPairCollision;
using hpp::core::continuousCollisionChecking::BodyPairCollisionPtr_t;

void display(const Model& model,
    const std::vector<JointIndex>& joints) {
  std::vector <JointIndex>::const_iterator it;
  for (it = joints.begin (); it != joints.end (); ++it) {
    if (*it > 0) std::cout << model.names[*it];
    else         std::cout << "None";
    std::cout << ", ";
  }
  std::cout << std::endl;
}


BOOST_AUTO_TEST_SUITE( test_hpp_core )

DevicePtr_t createRobot (){
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  return robot;
}

BOOST_AUTO_TEST_CASE (body_pair_collision_1)
{
  DevicePtr_t robot = createRobot();
  const Model& model = robot->model();
  JointPtr_t joint_a = robot->getJointByBodyName ("lleg5_body");
  JointPtr_t joint_b = robot->getJointByBodyName ("rleg5_body");

  BodyPairCollisionPtr_t bpc = BodyPairCollision::create
    (joint_a, joint_b, 0.001);
  display(model, bpc->joints());

  ConstObjectStdVector_t obstacles;
  fcl::CollisionGeometryPtr_t box (new fcl::Box (.2, .4, .6));
  // FIXME this is a bit ugly.
  int frame_id = robot->model().addFrame(::pinocchio::Frame("base_link",0,0,Transform3f (),::pinocchio::BODY));
  BOOST_CHECK(frame_id>=0);
  GeomIndex idObj = robot->geomModel().addGeometryObject
    (::pinocchio::GeometryObject("obstacle", frame_id, 0, box, Transform3f (), "", hpp::pinocchio::vector3_t::Ones()),
     robot->model());
  CollisionObjectPtr_t collObj (new hpp::pinocchio::CollisionObject(
        robot->geomModelPtr(), robot->geomDataPtr(), idObj));
  obstacles.push_back (collObj);
  bpc = BodyPairCollision::create (joint_a, obstacles, 0.001);
  display(model, bpc->joints());

  joint_a = robot->getJointByBodyName ("lleg5_body");
  joint_b = robot->getJointByBodyName ("lleg1_body");
  bpc = BodyPairCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->joints());

  joint_a = robot->getJointByBodyName ("lleg1_body");
  joint_b = robot->getJointByBodyName ("lleg5_body");
  bpc = BodyPairCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->joints());
}

BOOST_AUTO_TEST_SUITE_END()
