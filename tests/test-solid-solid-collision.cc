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
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/continuous-validation/solid-solid-collision.hh>
#include <hpp/core/relative-motion.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <limits>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

using hpp::core::continuousValidation::CoefficientVelocities_t;
using hpp::core::continuousValidation::Progressive;
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
  Transform3s I3;
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

BOOST_AUTO_TEST_CASE(virtual_parent) {
  DevicePtr_t robot = createRobot();
  JointPtr_t joint_a = robot->getJointByBodyName("lleg5_body");
  JointPtr_t joint_b = robot->getJointByBodyName("rleg1_body");
  JointPtr_t parent = robot->getJointByBodyName("lleg1_body");
  JointPtr_t root = robot->rootJoint();

  Transform3s identity;
  identity.setIdentity();
  auto box = coal::make_shared<coal::Box>(.2, .4, .6);
  GeomIndex id_a = robot->geomModel().addGeometryObject(
      ::pinocchio::GeometryObject("body_a", joint_a->index(),
                                  robot->model().getFrameId("lleg5_body"),
                                  identity, box),
      robot->model());
  GeomIndex id_b = robot->geomModel().addGeometryObject(
      ::pinocchio::GeometryObject("body_b", joint_b->index(),
                                  robot->model().getFrameId("rleg1_body"),
                                  identity, box),
      robot->model());
  robot->createGeomData();
  CollisionObjectPtr_t object_a(
      new CollisionObject(robot->geomModelPtr(), robot->geomDataPtr(), id_a));
  CollisionObjectPtr_t object_b(
      new CollisionObject(robot->geomModelPtr(), robot->geomDataPtr(), id_b));

  SolidSolidCollisionPtr_t collision =
      SolidSolidCollision::create(joint_a, joint_b, 0.001);
  collision->addCollisionPair(object_a, object_b);
  const CollisionPairs_t pairs(collision->pairs());
  const value_type radius = joint_a->linkedBody()->radius();
  const value_type offset = .7;
  BOOST_REQUIRE_GT(radius, 0);

  collision->setVirtualParent(joint_a, parent, offset);

  BOOST_CHECK(collision->joint_a() == joint_a);
  BOOST_CHECK(collision->joint_b() == joint_b);
  BOOST_CHECK_EQUAL(collision->indexJointA(), joint_a->index());
  BOOST_CHECK_EQUAL(collision->indexJointB(), joint_b->index());
  BOOST_REQUIRE_EQUAL(collision->pairs().size(), pairs.size());
  BOOST_CHECK(collision->pairs()[0].first == pairs[0].first);
  BOOST_CHECK(collision->pairs()[0].second == pairs[0].second);

  const CoefficientVelocities_t& coefficients(collision->coefficients());
  BOOST_REQUIRE_EQUAL(coefficients.size(), 2);
  BOOST_CHECK_EQUAL(coefficients[0].joint_->index(), parent->index());
  BOOST_CHECK_SMALL(
      coefficients[0].value_ -
          (parent->upperBoundLinearVelocity() +
           (radius + offset) * parent->upperBoundAngularVelocity()),
      1e-12);
  BOOST_CHECK_EQUAL(coefficients[1].joint_->index(), joint_b->index());
  BOOST_CHECK_SMALL(coefficients[1].value_ -
                        (joint_b->upperBoundLinearVelocity() +
                         (radius + offset + parent->maximalDistanceToParent()) *
                             joint_b->upperBoundAngularVelocity()),
                    1e-12);

  collision->setVirtualParent(joint_a, JointConstPtr_t(), offset);

  BOOST_CHECK(collision->joint_a() == joint_a);
  BOOST_CHECK(collision->joint_b() == joint_b);
  BOOST_REQUIRE_EQUAL(collision->pairs().size(), pairs.size());
  BOOST_CHECK(collision->pairs()[0].first == pairs[0].first);
  BOOST_CHECK(collision->pairs()[0].second == pairs[0].second);

  const CoefficientVelocities_t& universeCoefficients(
      collision->coefficients());
  BOOST_REQUIRE_EQUAL(universeCoefficients.size(), 2);
  BOOST_CHECK_EQUAL(universeCoefficients[0].joint_->index(), root->index());
  BOOST_CHECK_SMALL(universeCoefficients[0].value_ -
                        (root->upperBoundLinearVelocity() +
                         (radius + offset) * root->upperBoundAngularVelocity()),
                    1e-12);
  BOOST_CHECK_EQUAL(universeCoefficients[1].joint_->index(), joint_b->index());
  BOOST_CHECK_SMALL(universeCoefficients[1].value_ -
                        (joint_b->upperBoundLinearVelocity() +
                         (radius + offset + root->maximalDistanceToParent()) *
                             joint_b->upperBoundAngularVelocity()),
                    1e-12);
}

BOOST_AUTO_TEST_CASE(security_margin_between_bodies) {
  DevicePtr_t robot = createRobot();
  JointPtr_t joint_a = robot->getJointByBodyName("lleg5_body");
  JointPtr_t joint_b = robot->getJointByBodyName("rleg1_body");

  Transform3s identity;
  identity.setIdentity();
  auto box = coal::make_shared<coal::Box>(.2, .4, .6);
  GeomIndex id_a = robot->geomModel().addGeometryObject(
      ::pinocchio::GeometryObject("body_a", joint_a->index(),
                                  robot->model().getFrameId("lleg5_body"),
                                  identity, box),
      robot->model());
  GeomIndex id_b = robot->geomModel().addGeometryObject(
      ::pinocchio::GeometryObject("body_b", joint_b->index(),
                                  robot->model().getFrameId("rleg1_body"),
                                  identity, box),
      robot->model());
  robot->createGeomData();
  CollisionObjectPtr_t object_a(
      new CollisionObject(robot->geomModelPtr(), robot->geomDataPtr(), id_a));
  CollisionObjectPtr_t object_b(
      new CollisionObject(robot->geomModelPtr(), robot->geomDataPtr(), id_b));
  SolidSolidCollisionPtr_t collision =
      SolidSolidCollision::create(joint_a, joint_b, 0.001);
  collision->addCollisionPair(object_a, object_b);

  auto validation = Progressive::create(robot, 0.001);
  validation->addIntervalValidation(collision);
  RelativeMotion::matrix_type relativeMotion(RelativeMotion::matrix(robot));
  relativeMotion(joint_a->index(), joint_b->index()) =
      RelativeMotion::Constrained;
  relativeMotion(joint_b->index(), joint_a->index()) =
      RelativeMotion::Constrained;
  validation->filterCollisionPairs(relativeMotion);

  const value_type margin = 0.2;
  validation->setSecurityMarginBetweenBodies("body_a", "body_b", margin);
  BOOST_CHECK_EQUAL(collision->requests()[0].security_margin, margin);
  BOOST_CHECK_THROW(validation->setSecurityMarginBetweenBodies(
                        "unknown_a", "unknown_b", margin),
                    std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
