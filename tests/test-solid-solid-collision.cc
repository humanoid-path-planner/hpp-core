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

#define BOOST_TEST_MODULE solid_solid_collision

#include <boost/test/included/unit_test.hpp>

#include <limits>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/core/continuous-validation/solid-solid-collision.hh>

using std::numeric_limits;
using hpp::pinocchio::BodyPtr_t;
using hpp::pinocchio::CollisionObject;
using hpp::pinocchio::CollisionObjectPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::JointConstPtr_t;
using hpp::pinocchio::ObjectVector_t;
using hpp::core::continuousValidation::SolidSolidCollision;
using hpp::core::continuousValidation::SolidSolidCollisionPtr_t;
using hpp::core::continuousValidation::CoefficientVelocities_t;

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

void display(const se3::Model& model, const CoefficientVelocities_t& cvs) {
  CoefficientVelocities_t::const_iterator it;
  for (it = cvs.begin (); it != cvs.end (); ++it) {
    size_type jidx = it->joint_->index();
    if (jidx > 0) std::cout << model.names[jidx];
    else         std::cout << "None";
    std::cout << ", ";
  }
  std::cout << std::endl;
}


BOOST_AUTO_TEST_SUITE( test_hpp_core )

DevicePtr_t createRobot (){
  DevicePtr_t robot = hpp::pinocchio::humanoidSimple("test");
  return robot;
}


BOOST_AUTO_TEST_CASE (solid_solid_collision_1)
{
  DevicePtr_t robot = createRobot();
  const se3::Model& model = robot->model();
  JointPtr_t joint_a = robot->getJointByBodyName ("lleg5_body");
  JointPtr_t joint_b = robot->getJointByBodyName ("rleg5_body");

  SolidSolidCollisionPtr_t bpc = SolidSolidCollision::create
    (joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());

  ConstObjectStdVector_t obstacles;
  fcl::CollisionGeometryPtr_t box (new fcl::Box (.2, .4, .6));
  // FIXME this is a bit ugly.
  int frame_id = robot->model().addFrame(se3::Frame("base_link",0,0,Transform3f (),se3::BODY));
  BOOST_CHECK(frame_id>=0);
  ::se3::GeomIndex idObj = robot->geomModel().addGeometryObject
    (se3::GeometryObject("obstacle", frame_id, 0, box, Transform3f (), "", hpp::pinocchio::vector3_t::Ones()),
     robot->model());
  CollisionObjectPtr_t collObj (new hpp::pinocchio::CollisionObject(
        robot->geomModelPtr(), robot->geomDataPtr(), idObj));
  obstacles.push_back (collObj);
  bpc = SolidSolidCollision::create (joint_a, obstacles, 0.001);
  display(model, bpc->coefficients());

  joint_a = robot->getJointByBodyName ("lleg5_body");
  joint_b = robot->getJointByBodyName ("lleg1_body");
  bpc = SolidSolidCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());

  joint_a = robot->getJointByBodyName ("lleg1_body");
  joint_b = robot->getJointByBodyName ("lleg5_body");
  bpc = SolidSolidCollision::create (joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());
}

BOOST_AUTO_TEST_SUITE_END()
