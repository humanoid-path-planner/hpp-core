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

#define BOOST_TEST_MODULE gradient_based

#include <cmath>
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/model/device.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/path-optimization/gradient-based.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <pinocchio/multibody/joint/joint-variant.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include "../tests/utils.hh"


using hpp::pinocchio::BodyPtr_t;
using hpp::pinocchio::Body;
using hpp::pinocchio::CollisionObject;
using hpp::pinocchio::CollisionObjectPtr_t;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::Transform3f;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::value_type;
using fcl::Quaternion3f;
using fcl::Box;
using hpp::core::ConfigurationPtr_t;
using hpp::core::PathVector;
using hpp::core::PathVectorPtr_t;
using hpp::core::Problem;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::SteeringMethodStraight;
using hpp::core::PathOptimizerPtr_t;
using hpp::core::pathOptimization::GradientBased;

using namespace hpp::core;
using namespace hpp::pinocchio;

using ::se3::JointModelPX;
using ::se3::JointModelPY;
using ::se3::JointModelPZ;
using ::se3::JointModelRUBZ;
using ::se3::JointIndex;
using ::se3::FrameIndex;


BOOST_AUTO_TEST_SUITE( test_hpp_core )

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

  JointIndex idX = robot->model().addJoint(0,JointModelPX(), mat,jointName);

  std::string jointNameY = name + "_y";

  idX = robot->model().addJoint(idX,JointModelPY(), mat,jointNameY);

  idX = robot->model().addJoint(idX,JointModelRUBZ(), mat,"rot_z");

  fcl::Transform3f position; position.setIdentity ();

  boost::shared_ptr <Box> box (new Box (1,2,1));
  fcl::CollisionObject object(box, position);
  robot->model().appendBodyToJoint(idX,::se3::Inertia::Identity(), se3::SE3::Identity());
  robot->model().addBodyFrame("body", idX, se3::SE3::Identity());
  FrameIndex bodyId = robot->model().getFrameId("body");
  robot->geomModel().addGeometryObject(
      se3::GeometryObject("obstacle", bodyId, idX, object.collisionGeometry(), mat, "", hpp::pinocchio::vector3_t::Ones()),
      robot->model());

  robot->createData();
  robot->createGeomData();
  return robot;
  /*
  DevicePtr_t robot = Device::create ("planar-robot");
  Transform3f position; position.setIdentity ();
  ObjectFactory factory;

  // planar root joint
  JointPtr_t root = factory.createJointTranslation2 (position);
  robot->rootJoint (root);
  // Rotation around z
  position.setQuatRotation (Quaternion3f (sqrt (2)/2, 0, -sqrt (2)/2, 0));
  JointPtr_t joint = factory.createUnBoundedJointRotation (position);
  root->addChildJoint (joint);
  position.setIdentity ();
  boost::shared_ptr <Box> box (new Box (1,2,1));
  CollisionObjectPtr_t object = CollisionObject::create (box, position, "box");
  BodyPtr_t body = new Body ();
  joint->setLinkedBody (body);
  body->addInnerObject (object, true, true);

  return robot;*/
}

BOOST_AUTO_TEST_CASE (BFGS)
{
  DevicePtr_t robot = createRobot ();
  Configuration_t q0 (robot->configSize ());
  Configuration_t q1 (robot->configSize ());
  Configuration_t q2 (robot->configSize ());
  Configuration_t q3 (robot->configSize ());
  Configuration_t q4 (robot->configSize ());
  value_type s = sqrt (2)/2;
  q0 (0) = -1; q0 (1) = 0; q0 (2) = 1; q0 (3) = 0;
  q1 (0) = -s; q1 (1) = s; q1 (2) = s; q1 (3) = s;
  q2 (0) = 0; q2 (1) = 1; q2 (2) = 0; q2 (3) = 1;
  q3 (0) = s; q3 (1) = s; q3 (2) = s; q3 (3) = s;
  q4 (0) = 1; q4 (1) = 0; q4 (2) = 1; q4 (3) = 0;

  Problem problem (robot);
  SteeringMethodPtr_t sm = problem.steeringMethod ();
  PathVectorPtr_t path = PathVector::create (robot->configSize (),
					     robot->numberDof ());
  path->appendPath ((*sm) (q0, q1));
  path->appendPath ((*sm) (q1, q2));
  path->appendPath ((*sm) (q2, q3));
  path->appendPath ((*sm) (q3, q4));

  PathOptimizerPtr_t pathOptimizer (GradientBased::create (problem));
  pathOptimizer->optimize (path);
}
BOOST_AUTO_TEST_SUITE_END()
