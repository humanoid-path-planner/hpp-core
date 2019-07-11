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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/implicit.hh>

using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::matrix3_t;
using hpp::constraints::vector3_t;

using namespace hpp;
using namespace hpp::pinocchio;
using namespace hpp::core;

typedef Eigen::Matrix<value_type,3,1> Vector3;
typedef Eigen::Matrix<value_type,3,3> Matrix3;

DevicePtr_t createRobot (){
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  const value_type l = 2;
  robot->rootJoint()->lowerBound(0, -l);
  robot->rootJoint()->lowerBound(1, -l);
  robot->rootJoint()->lowerBound(2, -l);
  robot->rootJoint()->upperBound(0,  l);
  robot->rootJoint()->upperBound(1,  l);
  robot->rootJoint()->upperBound(2,  l);
  return robot;
}

BOOST_AUTO_TEST_SUITE (config_projector)

BOOST_AUTO_TEST_CASE (empty)
{
  DevicePtr_t dev = createRobot();

  Configuration_t cfg(dev->neutralConfiguration ());
  ConstraintSetPtr_t cs = ConstraintSet::create (dev, "empty");
  BOOST_CHECK(cs->apply(cfg));

  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);
  cs->addConstraint (projector);
  BOOST_CHECK (projector->apply (cfg));
}

BOOST_AUTO_TEST_CASE (ref_zero)
{
  DevicePtr_t dev = createRobot();
  JointPtr_t xyz = dev->getJointByName ("root_joint");
  matrix3_t rot; rot.setIdentity ();
  vector3_t zero; zero.setZero();
  ComparisonTypes_t ineq (3, constraints::Superior);
  BOOST_REQUIRE (dev);
  PositionPtr_t position =
    Position::create ("Position", dev, xyz, Transform3f(rot,zero));

  ineq[1] = constraints::Inferior;
  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);

  projector->add (constraints::Implicit::create (position, ineq));
  // Constraints are
  // q [0] > 0
  // q [1] < 0
  // q [2] > 0
  Configuration_t cfg(dev->neutralConfiguration ());

  vector_t invert(3);
  invert[0] =  1;
  invert[1] = -1;
  invert[2] =  1;
  cfg.segment (0,3) = invert;
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ((cfg.segment (0, 3) - invert).isZero (), "Configuration should not be modified.");

  cfg.segment (0,3) = vector_t::Ones (3);
  BOOST_CHECK (projector->apply (cfg));
  BOOST_CHECK_MESSAGE ((cfg.segment (0,1) - vector_t::Ones(1)).isZero (), "Dof 0 should not have been modified. " << cfg.head<3>().transpose());
  BOOST_CHECK_MESSAGE ( cfg (1) < 0                                     , "Dof 1 should have been modified. " << cfg.head<3>().transpose());
  BOOST_CHECK_MESSAGE ((cfg.segment (2,1) - vector_t::Ones(1)).isZero (), "Dof 2 should not have been modified. " << cfg.head<3>().transpose());
}

BOOST_AUTO_TEST_CASE (ref_not_zero)
{
  DevicePtr_t dev = createRobot();
  JointPtr_t xyz = dev->getJointByName ("root_joint");
  matrix3_t rot; rot.setIdentity ();
  vector3_t zero; zero.setZero();
  vector_t ref(3);
  ComparisonTypes_t ineq (3, constraints::Superior);
  BOOST_REQUIRE (dev);
  PositionPtr_t position =
    Position::create ("Position", dev, xyz, Transform3f(rot,zero), Transform3f(rot,vector3_t (1,1,1)));

  ref[0] = 0; ref[1] = 0; ref[2] = 0;
  ineq[1] = constraints::Inferior;
  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);
  projector->add (constraints::Implicit::create (position, ineq));
  Configuration_t cfg(dev->neutralConfiguration ());

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
  BOOST_CHECK_MESSAGE ( cfg (0) > 1                                     , "Dof 0 should have been modified. " << cfg.head<3>().transpose());
  BOOST_CHECK_MESSAGE ((cfg.segment (1,1) - ref.segment (1,1)).isZero (), "Dof 1 should not have been modified. " << cfg.head<3>().transpose());
  BOOST_CHECK_MESSAGE ( cfg (2) > 1                                     , "Dof 2 should have been modified. " << cfg.head<3>().transpose());
}

BOOST_AUTO_TEST_CASE (copy)
{
  using constraints::Implicit;
  using constraints::ImplicitPtr_t;
  DevicePtr_t dev = createRobot();
  JointPtr_t xyz = dev->getJointByName ("root_joint");
  ConfigProjectorPtr_t projector =
    ConfigProjector::create (dev, "test", 1e-4, 20);
  matrix3_t rot; rot.setIdentity ();
  vector3_t zero; zero.setZero();
  PositionPtr_t position
    (Position::create ("Position", dev, xyz, Transform3f (rot, zero),
                       Transform3f(rot,vector3_t (1,1,1))));
  ComparisonTypes_t equality (3, constraints::Equality);
  ImplicitPtr_t constraint (Implicit::create (position, equality));
  projector->add (constraint);

  ConfigProjectorPtr_t copy (HPP_DYNAMIC_PTR_CAST (ConfigProjector,
                                                   projector->copy ()));
  vector_t rhs (3);
  bool success (copy->solver ().getRightHandSide (constraint, rhs));
  BOOST_CHECK (success);
}

BOOST_AUTO_TEST_SUITE_END()
