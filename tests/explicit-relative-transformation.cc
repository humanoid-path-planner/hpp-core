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
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/constraints/differentiable-function.hh>

using namespace hpp::pinocchio;
using namespace hpp::constraints;
using namespace hpp::core;

DevicePtr_t createRobot ()
{
  //DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidRomeo, "romeo");
  DevicePtr_t robot (Device::create ("2-objects"));
  urdf::loadModel<false> (robot, 0, "obj1/", "freeflyer", "file://" DATA_DIR "/empty.urdf", "");
  robot->controlComputation((Device::Computation_t) (Device::JOINT_POSITION | Device::JACOBIAN));
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->lowerBound (2, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);
  robot->rootJoint()->upperBound (2,  10);

  /// Add a freeflyer at the end.
  urdf::loadModel<false> (robot, 0, "obj2/", "freeflyer", "file://" DATA_DIR "/empty.urdf", "");
  JointPtr_t rj = robot->getJointByName("obj2/root_joint");
  rj->lowerBound (0, -10);
  rj->lowerBound (1, -10);
  rj->lowerBound (2, -10);
  rj->upperBound (0,  10);
  rj->upperBound (1,  10);
  rj->upperBound (2,  10);

  return robot;
}

template <typename T> inline typename T::template NRowsBlockXpr<3>::Type trans(const Eigen::MatrixBase<T>& j) { return const_cast<Eigen::MatrixBase<T>&>(j).derived().template topRows   <3>(); }
template <typename T> inline typename T::template NRowsBlockXpr<3>::Type omega(const Eigen::MatrixBase<T>& j) { return const_cast<Eigen::MatrixBase<T>&>(j).derived().template bottomRows<3>(); }

BOOST_AUTO_TEST_CASE (two_freeflyer)
{
  DevicePtr_t robot = createRobot();
  BOOST_REQUIRE (robot);

  JointPtr_t object2 = robot->getJointByName("obj2/root_joint");
  JointPtr_t object1  = robot->getJointByName("obj1/root_joint");

  Transform3f M2inO2 (Transform3f::Identity());
  Transform3f M1inO1 (Transform3f::Identity());

  ExplicitRelativeTransformationPtr_t ert = ExplicitRelativeTransformation::create (
      "explicit_relative_transformation", robot,
      object1, object2, M1inO1, M2inO2);
  ExplicitNumericalConstraintPtr_t enm = ert->createNumericalConstraint();

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = se3::randomConfiguration(robot->model()),
                  qout = qrand;

  // Check the output value
  Eigen::RowBlockIndices outConf (enm->outputConf());
  Eigen::RowBlockIndices  inConf (enm-> inputConf());

  // Compute position of object2 by solving explicit constraints
  LiegroupElement q_obj2 (ert->outputSpace ());
  vector_t q_obj1 (inConf.rview(qout).eval());
  ert->value (q_obj2, q_obj1);
  outConf.lview(qout) = q_obj2.vector ();

  // Test that at solution configuration, object2 and robot frames coincide.
  robot->currentConfiguration(qout);
  robot->computeForwardKinematics();
  Transform3f diff =
    M1inO1.inverse() * object1->currentTransformation().inverse()
    * object2->currentTransformation() * M2inO2;

  BOOST_CHECK (diff.isIdentity());

  // Check Jacobian of implicit numerical constraints by finite difference
  //
  value_type dt (1e-5);
  Configuration_t q0 (qout);
  vector_t v (robot->numberDof ());
  matrix_t J (6, enm->function ().inputDerivativeSize());
  LiegroupElement value0 (enm->function ().outputSpace ()),
    value (enm->function ().outputSpace ());
  enm->function ().value (value0, q0);
  enm->function ().jacobian (J, q0);
    std::cout << "J=" << std::endl << J << std::endl;
    std::cout << "q0=" << q0.transpose () << std::endl;
  // First at solution configuration
  for (size_type i=0; i<12; ++i) {
    // test canonical basis vectors
    v.setZero (); v [i] = 1;
    integrate (robot, q0, dt * v, q);
    enm->function ().value (value, q);
    vector_t df ((value - value0)/dt);
    vector_t Jdq (J * v);
    std::cout << "v=" << v.transpose () << std::endl;
    std::cout << "q=" << q.transpose () << std::endl;
    std::cout << "df=" << df.transpose () << std::endl;
    std::cout << "Jdq=" << Jdq.transpose () << std::endl;
    std::cout << "||Jdq - df ||=" << (df - Jdq).norm () << std::endl << std::endl;
    BOOST_CHECK ((df - Jdq).norm () < 1e-4);
  }
  // Second at random configurations
  for (size_type i=0; i<100; ++i) {
    q0 = se3::randomConfiguration(robot->model());
    enm->function ().jacobian (J, q0);
    std::cout << "J=" << std::endl << J << std::endl;
    std::cout << "q0=" << q0.transpose () << std::endl;
    enm->function ().value (value0, q0);
    enm->function ().jacobian (J, q0);
    for (size_type j=0; j<12; ++j) {
      v.setZero (); v [j] = 1;
      std::cout << "v=" << v.transpose () << std::endl;
      integrate (robot, q0, dt * v, q);
      std::cout << "q=" << q.transpose () << std::endl;
      enm->function ().value (value, q);
      vector_t df ((value - value0)/dt);
      vector_t Jdq (J * v);
      std::cout << "df=" << df.transpose () << std::endl;
      std::cout << "Jdq=" << Jdq.transpose () << std::endl;
      std::cout << "||Jdq - df ||=" << (df - Jdq).norm () << std::endl << std::endl;
      BOOST_CHECK ((df - Jdq).norm () < 1e-4);
    }
    std::cout << std::endl;
  }
}

BOOST_AUTO_TEST_CASE (two_frames_on_freeflyer)
{
  DevicePtr_t robot = createRobot();
  BOOST_REQUIRE (robot);

  JointPtr_t object2 = robot->getJointByName("obj2/root_joint");
  JointPtr_t object1  = robot->getJointByName("obj1/root_joint");

  Transform3f M2inO2 (Transform3f::Random ());
  Transform3f M1inO1 (Transform3f::Random ());

  std::cout << "M2inO2=" << M2inO2 << std::endl;
  std::cout << "M1inO1=" << M1inO1 << std::endl;

  ExplicitRelativeTransformationPtr_t ert = ExplicitRelativeTransformation::create (
      "explicit_relative_transformation", robot,
      object1, object2, M1inO1, M2inO2);
  ExplicitNumericalConstraintPtr_t enm = ert->createNumericalConstraint();

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = se3::randomConfiguration(robot->model()),
                  qout = qrand;

  // Check the output value
  Eigen::RowBlockIndices outConf (enm->outputConf());
  Eigen::RowBlockIndices  inConf (enm-> inputConf());

  // Compute position of object2 by solving explicit constraints
  LiegroupElement q_obj2 (ert->outputSpace ());
  vector_t q_obj1 (inConf.rview(qout).eval());
  ert->value (q_obj2, q_obj1);
  outConf.lview(qout) = q_obj2.vector ();

  // Test that at solution configuration, object2 and robot frames coincide.
  robot->currentConfiguration(qout);
  robot->computeForwardKinematics();
  Transform3f diff =
    M1inO1.inverse() * object1->currentTransformation().inverse()
    * object2->currentTransformation() * M2inO2;

  BOOST_CHECK (diff.isIdentity());

  // Check Jacobian of implicit numerical constraints by finite difference
  //
  value_type dt (1e-5);
  Configuration_t q0 (qout);
  vector_t v (robot->numberDof ());
  matrix_t J (6, enm->function ().inputDerivativeSize());
  LiegroupElement value0 (enm->function ().outputSpace ()),
    value (enm->function ().outputSpace ());
  enm->function ().value (value0, q0);
  enm->function ().jacobian (J, q0);
    std::cout << "J=" << std::endl << J << std::endl;
    std::cout << "q0=" << q0.transpose () << std::endl;
  // First at solution configuration
  for (size_type i=0; i<12; ++i) {
    // test canonical basis vectors
    v.setZero (); v [i] = 1;
    integrate (robot, q0, dt * v, q);
    enm->function ().value (value, q);
    vector_t df ((value - value0)/dt);
    vector_t Jdq (J * v);
    std::cout << "v=" << v.transpose () << std::endl;
    std::cout << "q=" << q.transpose () << std::endl;
    std::cout << "df=" << df.transpose () << std::endl;
    std::cout << "Jdq=" << Jdq.transpose () << std::endl;
    std::cout << "||Jdq - df ||=" << (df - Jdq).norm () << std::endl << std::endl;
    BOOST_CHECK ((df - Jdq).norm () < 1e-4);
  }
  // Second at random configurations
  for (size_type i=0; i<100; ++i) {
    q0 = se3::randomConfiguration(robot->model());
    enm->function ().jacobian (J, q0);
    std::cout << "J=" << std::endl << J << std::endl;
    std::cout << "q0=" << q0.transpose () << std::endl;
    enm->function ().value (value0, q0);
    enm->function ().jacobian (J, q0);
    for (size_type j=0; j<12; ++j) {
      v.setZero (); v [j] = 1;
      std::cout << "v=" << v.transpose () << std::endl;
      integrate (robot, q0, dt * v, q);
      std::cout << "q=" << q.transpose () << std::endl;
      enm->function ().value (value, q);
      vector_t df ((value - value0)/dt);
      vector_t Jdq (J * v);
      std::cout << "df=" << df.transpose () << std::endl;
      std::cout << "Jdq=" << Jdq.transpose () << std::endl;
      std::cout << "||Jdq - df ||=" << (df - Jdq).norm () << std::endl << std::endl;
      BOOST_CHECK ((df - Jdq).norm () < 1e-4);
    }
    std::cout << std::endl;
  }
}
