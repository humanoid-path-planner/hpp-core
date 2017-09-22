// Copyright (c) 2017, Joseph Mirabel
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

#define BOOST_TEST_MODULE ExplicitRelativeTransformationTest
#include <boost/test/included/unit_test.hpp>

#include <boost/shared_ptr.hpp>

// Force benchmark output
#define HPP_ENABLE_BENCHMARK 1
#include <hpp/util/timer.hh>

#include <hpp/core/fwd.hh>
#include <hpp/core/explicit-relative-transformation.hh>

#include <pinocchio/spatial/skew.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>

#include <hpp/constraints/differentiable-function.hh>

using namespace hpp::pinocchio;
using namespace hpp::constraints;
using namespace hpp::core;

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidRomeo);
  robot->controlComputation((Device::Computation_t) (Device::JOINT_POSITION | Device::JACOBIAN));
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->lowerBound (2, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);
  robot->rootJoint()->upperBound (2,  10);

  /// Add a freeflyer at the end.
  urdf::loadModel<false> (robot, 0, "empty/", "freeflyer", "file://" DATA_DIR "/empty.urdf", "");
  JointPtr_t rj = robot->getJointByName("empty/root_joint");
  rj->lowerBound (0, -1);
  rj->lowerBound (1, -1);
  rj->lowerBound (2, -1);
  rj->upperBound (0,  1);
  rj->upperBound (1,  1);
  rj->upperBound (2,  1);

  return robot;
}

template <typename T> inline typename T::template NRowsBlockXpr<3>::Type trans(const Eigen::MatrixBase<T>& j) { return const_cast<Eigen::MatrixBase<T>&>(j).derived().template topRows   <3>(); }
template <typename T> inline typename T::template NRowsBlockXpr<3>::Type omega(const Eigen::MatrixBase<T>& j) { return const_cast<Eigen::MatrixBase<T>&>(j).derived().template bottomRows<3>(); }

BOOST_AUTO_TEST_CASE (explicit_relative_transformation)
{
  DevicePtr_t robot = createRobot();
  BOOST_REQUIRE (robot);

  JointPtr_t object = robot->getJointByName("empty/root_joint");
  JointPtr_t joint  = robot->getJointByName("LWristPitch");

  Transform3f aMt (Transform3f::Identity());
  Transform3f jMt (Transform3f::Random());

  ExplicitRelativeTransformationPtr_t ert = ExplicitRelativeTransformation::create (
      "explicit_relative_transformation", robot,
      joint, object, jMt, aMt);
  ExplicitNumericalConstraintPtr_t enm = ert->createNumericalConstraint();

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = se3::randomConfiguration(robot->model()),
                  qout = qrand;

  // Check the output value
  Eigen::RowBlockIndices outConf (enm->outputConf());
  Eigen::RowBlockIndices  inConf (enm-> inputConf());

  vector_t tmp(outConf.nbIndices());
  ert->value (tmp, inConf.rview(qrand).eval());
  outConf.lview(qout) = tmp;

  robot->currentConfiguration(qout);
  robot->computeForwardKinematics();
  Transform3f diff =
    jMt.inverse() * joint->currentTransformation().inverse()
    * object->currentTransformation() * aMt;

  BOOST_CHECK (diff.isIdentity());

  // Check the jacobian
  Eigen::RowBlockIndices outVel (enm->outputVelocity());
  Eigen::RowBlockIndices  inVel (enm-> inputVelocity());

  // The velocity of the target in wrist joint should be the same as
  // the velocity of the target in the object joint.
  const Transform3f& oMj    = joint->currentTransformation();
  const JointJacobian_t& Jj = joint->jacobian();
  const Transform3f& oMa    = object->currentTransformation();
  const JointJacobian_t& Ja = object->jacobian();

  matrix_t oJjt (6, robot->numberDof());
  trans(oJjt) = ( se3::skew( (- oMj.rotation() * jMt.translation()).eval() ) * oMj.rotation() * omega(Jj) + oMj.rotation() * trans(Jj) );
  omega(oJjt) = oMj.rotation() * omega(Jj);

  matrix_t oJat (6, robot->numberDof());
  trans(oJat) = ( se3::skew( (- oMa.rotation() * aMt.translation()).eval() ) * oMa.rotation() * omega(Ja) + oMa.rotation() * trans(Ja) );
  omega(oJat) = oMa.rotation() * omega(Ja);

  matrix_t Jert (6, ert->inputDerivativeSize());
  // ert->jacobian (Jert, inConf.rview(qout).eval());
  ert->jacobian (Jert, inConf.rview(qrand).eval());

  vector_t qdot (robot->numberDof()), qdot_out;

  qdot.setRandom();
  qdot_out = qdot;

  vector_t oVjt = oJjt * qdot;
  // std::cout << oVjt.transpose() << std::endl;

  outVel.lview (qdot_out) = Jert * inVel.rview(qdot).eval();

  vector_t oVat = oJat * qdot_out;
  // std::cout << oVat.transpose() << std::endl;

  BOOST_CHECK(oVjt.isApprox(oVat));
}
