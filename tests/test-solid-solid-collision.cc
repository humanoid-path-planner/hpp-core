//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <coal/fwd.hh>
#define BOOST_TEST_MODULE solid_solid_collision
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>

#include <boost/test/included/unit_test.hpp>
#include <hpp/core/continuous-validation/solid-solid-collision.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <limits>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

using hpp::core::continuousValidation::CoefficientVelocities_t;
using hpp::core::continuousValidation::SolidSolidCollision;
using hpp::core::continuousValidation::SolidSolidCollisionPtr_t;
using hpp::pinocchio::BodyPtr_t;
using hpp::pinocchio::CollisionObject;
using hpp::pinocchio::CollisionObjectPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointConstPtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::ObjectVector_t;
using std::numeric_limits;

using namespace hpp::core;
using namespace hpp::pinocchio;

void display(const Model& model, const CoefficientVelocities_t& cvs) {
  CoefficientVelocities_t::const_iterator it;
  for (it = cvs.begin(); it != cvs.end(); ++it) {
    size_type jidx = it->joint_->index();
    if (jidx > 0)
      std::cout << model.names[jidx];
    else
      std::cout << "None";
    std::cout << ", ";
  }
  std::cout << std::endl;
}

BOOST_AUTO_TEST_SUITE(test_hpp_core)

DevicePtr_t createRobot() {
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  return robot;
}

BOOST_AUTO_TEST_CASE(solid_solid_collision_1) {
  DevicePtr_t robot = createRobot();
  const Model& model = robot->model();
  JointPtr_t joint_a = robot->getJointByBodyName("lleg5_body");
  JointPtr_t joint_b = robot->getJointByBodyName("rleg5_body");

  SolidSolidCollisionPtr_t bpc =
      SolidSolidCollision::create(joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());

  ConstObjectStdVector_t obstacles;
  auto box = coal::make_shared<coal::Box>(.2, .4, .6);
  Transform3f I3;
  I3.setIdentity();
  pinocchio::FrameIndex frame_id = robot->model().addFrame(
      ::pinocchio::Frame("base_link", 0, 0, I3, ::pinocchio::BODY));
  GeomIndex idObj = robot->geomModel().addGeometryObject(
      ::pinocchio::GeometryObject("obstacle", frame_id, 0, box, I3, "",
                                  vector3_t::Ones()),
      robot->model());
  CollisionObjectPtr_t collObj(
      new CollisionObject(robot->geomModelPtr(), robot->geomDataPtr(), idObj));
  obstacles.push_back(collObj);
  bpc = SolidSolidCollision::create(joint_a, obstacles, 0.001);
  display(model, bpc->coefficients());

  joint_a = robot->getJointByBodyName("lleg5_body");
  joint_b = robot->getJointByBodyName("lleg1_body");
  bpc = SolidSolidCollision::create(joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());

  joint_a = robot->getJointByBodyName("lleg1_body");
  joint_b = robot->getJointByBodyName("lleg5_body");
  bpc = SolidSolidCollision::create(joint_a, joint_b, 0.001);
  display(model, bpc->coefficients());
}

BOOST_AUTO_TEST_SUITE_END()
