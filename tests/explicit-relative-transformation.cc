// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE ExplicitRelativeTransformationTest
#include <pinocchio/fwd.hpp>
#include <boost/test/included/unit_test.hpp>

#include <../tests/util.hh>

// Force benchmark output
#define HPP_ENABLE_BENCHMARK 1
#include <hpp/util/timer.hh>

#include <hpp/core/fwd.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/constraints/explicit/relative-pose.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/generic-transformation.hh>

#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>

using namespace hpp::pinocchio;
using namespace hpp::constraints;
using namespace hpp::core;

#define NB_RANDOM_CONF 2

const value_type test_precision = 1e-8;

DevicePtr_t createRobot ()
{
  //DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidRomeo, "romeo");
  DevicePtr_t robot (Device::create ("2-objects"));
  urdf::loadModel (robot, 0, "obj1/", "freeflyer", "file://" DATA_DIR "/empty.urdf", "");
  robot->controlComputation((Computation_t) (JOINT_POSITION | JACOBIAN));
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->lowerBound (2, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);
  robot->rootJoint()->upperBound (2,  10);

  /// Add a freeflyer at the end.
  urdf::loadModel (robot, 0, "obj2/", "freeflyer", "file://" DATA_DIR "/empty.urdf", "");
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

BOOST_AUTO_TEST_CASE (two_freeflyers)
{
  DevicePtr_t robot = createRobot();
  BOOST_REQUIRE (robot);
  ConfigurationShooterPtr_t cs = configurationShooter::Uniform::create(robot);

  JointPtr_t object2 = robot->getJointByName("obj2/root_joint");
  JointPtr_t object1  = robot->getJointByName("obj1/root_joint");

  Transform3f M2inO2 (Transform3f::Identity());
  Transform3f M1inO1 (Transform3f::Identity());

  ExplicitPtr_t enm (explicit_::RelativePose::create
                     ("explicit_relative_transformation", robot, object1,
                      object2, M1inO1, M2inO2, 6 * Equality));
  DifferentiableFunctionPtr_t ert (enm->explicitFunction ());

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = *cs->shoot(),
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
  for (size_type i=0; i<NB_RANDOM_CONF; ++i) {
    q0 = *cs->shoot();
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
  ConfigurationShooterPtr_t cs = configurationShooter::Uniform::create(robot);

  JointPtr_t object2 = robot->getJointByName("obj2/root_joint");
  JointPtr_t object1  = robot->getJointByName("obj1/root_joint");

  Transform3f M2inO2 (Transform3f::Random ());
  Transform3f M1inO1 (Transform3f::Random ());

  std::cout << "M2inO2=" << M2inO2 << std::endl;
  std::cout << "M1inO1=" << M1inO1 << std::endl;

  ExplicitPtr_t enm (explicit_::RelativePose::create
                     ("explicit_relative_transformation", robot, object1,
                      object2, M1inO1, M2inO2, 6 * Equality));
  DifferentiableFunctionPtr_t ert (enm->explicitFunction ());

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = *cs->shoot(),
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
  for (size_type i=0; i<NB_RANDOM_CONF; ++i) {
    q0 = *cs->shoot();
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

BOOST_AUTO_TEST_CASE (compare_to_relative_transform)
{
  DevicePtr_t robot = createRobot();
  BOOST_REQUIRE (robot);
  ConfigurationShooterPtr_t cs = configurationShooter::Uniform::create(robot);

  JointPtr_t object2 = robot->getJointByName("obj2/root_joint");
  JointPtr_t object1  = robot->getJointByName("obj1/root_joint");

  Transform3f M2inO2 (Transform3f::Random ());
  Transform3f M1inO1 (Transform3f::Random ());

  BOOST_TEST_MESSAGE("M2inO2=" << M2inO2);
  BOOST_TEST_MESSAGE("M1inO1=" << M1inO1);

  ExplicitPtr_t enm (explicit_::RelativePose::create
                     ("explicit_relative_transformation", robot, object1,
                      object2, M1inO1, M2inO2, 6 * Equality));
  DifferentiableFunctionPtr_t ert (enm->explicitFunction ());
  DifferentiableFunctionPtr_t irt = enm->functionPtr();
  RelativeTransformationR3xSO3::Ptr_t rt (RelativeTransformationR3xSO3::create(
      "relative_transformation_R3xSO3", robot,
      object1, object2, M1inO1, M2inO2));

  Configuration_t q     = robot->currentConfiguration (),
                  qrand = *cs->shoot(),
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

  // Check that value and Jacobian of relative transform from GenericTransformation
  // and from ExplicitRelativeTransformation are equal

  BOOST_CHECK_EQUAL (irt->inputSize(), rt->inputSize());
  BOOST_CHECK_EQUAL (irt->inputDerivativeSize(), rt->inputDerivativeSize());
  BOOST_CHECK_EQUAL (irt->outputSize(), rt->outputSize());
  BOOST_CHECK_EQUAL (irt->outputDerivativeSize(), rt->outputDerivativeSize());
  BOOST_CHECK_EQUAL (*irt->outputSpace(), *rt->outputSpace());

  Configuration_t q0 (qout);
  LiegroupElement v0 (rt->outputSpace ()), v1 (v0),
                  q_out (q_obj2), q_in  (q_obj2);
  matrix_t J0 (rt->outputSpace()->nv(), rt->inputDerivativeSize()), J1 (J0);

  irt->value    (v0, q0);
  irt->jacobian (J0, q0);
  rt ->value    (v1, q0);
  rt ->jacobian (J1, q0);

  BOOST_CHECK (log(v0).isZero(test_precision));
  BOOST_CHECK (log(v1).isZero(test_precision));
  EIGEN_IS_APPROX (J0, J1, test_precision);

  // Second at random configurations
  for (size_type i=0; i<NB_RANDOM_CONF; ++i) {
    q0 = *cs->shoot();

    irt->value    (v0, q0);
    irt->jacobian (J0, q0);
    rt ->value    (v1, q0);
    rt ->jacobian (J1, q0);
    EIGEN_VECTOR_IS_APPROX (v0.vector(), v1.vector(), test_precision);
    EIGEN_IS_APPROX (J0, J1, test_precision);
  }
}
