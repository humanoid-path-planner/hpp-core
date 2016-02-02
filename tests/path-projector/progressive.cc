// Copyright (c) 2016, Joseph Mirabel
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

#define BOOST_TEST_MODULE pathProjector_Progressive
#include <boost/test/included/unit_test.hpp>

#include <boost/shared_ptr.hpp>
// Boost version 1.54
// Cannot include boost CPU timers
// #include <boost/timer/timer.hpp>
// because the original timers are already included by
// the unit test framework
// #include <boost/timer.hh>

// Force benchmark output
#define HPP_ENABLE_BENCHMARK 1
#include <hpp/util/timer.hh>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/core/straight-path.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem.hh>

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;

using namespace hpp::core;

hpp::model::ObjectFactory objectFactory;

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("test");

  const std::string& name = robot->name ();
  fcl::Transform3f mat; mat.setIdentity ();
  JointPtr_t joint;
  std::string jointName = name + "_x";
  // Translation along x
  fcl::Matrix3f permutation;
  joint = objectFactory.createJointTranslation2 (mat);
  joint->name (jointName);

  joint->isBounded (0, 1);
  joint->lowerBound (0, -4);
  joint->upperBound (0, +4);
  joint->isBounded (1, 1);
  joint->lowerBound (1, -4);
  joint->upperBound (1, +4);

  robot->rootJoint (joint);
  return robot;
}

ConstraintSetPtr_t createConstraints (DevicePtr_t r)
{
  ConfigProjectorPtr_t proj =
    ConfigProjector::create (r, "Polynomial projector", 1e-4, 20);
  ConstraintSetPtr_t set = ConstraintSet::create (r, "Set");
  set->addConstraint (proj);
  return set;
}

class Polynomial : public DifferentiableFunction {
  public:
    Polynomial (DevicePtr_t robot) :
      DifferentiableFunction (robot->configSize(), robot->numberDof (), 1, "Polynomial"),
      coefs_ (vector_t::Ones (robot->configSize()))
    {}

    vector_t coefs_;

  protected:
      void impl_compute (vectorOut_t result, vectorIn_t argument) const {
        result[0] = argument.cwiseProduct (argument).dot (coefs_) - 1;
      }
      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const {
        jacobian.row(0) = 2 * arg.cwiseProduct (coefs_);
      }
};

typedef boost::shared_ptr<Polynomial> PolynomialPtr_t;

void displayPaths (PathPtr_t path, PathPtr_t projection, DifferentiableFunctionPtr_t func) {
  const value_type stepPath = path->length () / (100 - 1);
  const value_type stepProj = projection->length () / (100 - 1);
  Configuration_t q = path->initial(), qq = path->initial();
  vector_t v1 (func->outputSize()), v2(func->outputSize());
  std::cout << std::fixed << std::showpos << std::setprecision (4);
  const char* sep = "\t| ";
  for (std::size_t i = 0; i < 100; ++i) {
    if (!(*path) (q, i * stepPath))
      std::cerr << "Could not project path at " << i*stepPath << "\n";
    if (!(*projection) (qq, i * stepProj))
      std::cerr << "Could not project projection at " << i*stepProj << "\n";
    (*func) (v1, q);
    (*func) (v2, qq);
    std::cout << q.transpose () << sep << v1
      << sep << qq.transpose () << sep << v2 << "\n";
  }
}

HPP_DEFINE_TIMECOUNTER(progressiveCircle);

BOOST_AUTO_TEST_CASE (circle)
{
  DevicePtr_t dev = createRobot ();
  BOOST_REQUIRE (dev);
  Problem problem (dev);

  ConstraintSetPtr_t c = createConstraints (dev);
  PolynomialPtr_t circle (new Polynomial (dev));
  c->configProjector ()->add (NumericalConstraint::create (circle));
  problem.steeringMethod ()->constraints (c);

  pathProjector::ProgressivePtr_t progressive =
    pathProjector::Progressive::create (problem.distance(),
        problem.steeringMethod(), 0.1);

  Configuration_t q1 (dev->configSize()); q1 << 0,1;
  Configuration_t q2 (dev->configSize()); q2 << 1,0;
  PathPtr_t path = (*problem.steeringMethod ()) (q1,q2);

  PathPtr_t projection;
  // Averaging the projection
  bool success;
  for (int i = 0; i < 100; ++i) {
    HPP_START_TIMECOUNTER (progressiveCircle);
    bool success = progressive->apply (path, projection);
    HPP_STOP_TIMECOUNTER (progressiveCircle);
    HPP_DISPLAY_LAST_TIMECOUNTER (progressiveCircle);
  }
  HPP_DISPLAY_TIMECOUNTER (progressiveCircle);
  std::cout << success << std::endl;
  displayPaths (path, projection, circle);
}

HPP_DEFINE_TIMECOUNTER(progressiveParabola);

BOOST_AUTO_TEST_CASE (parabola)
{
  DevicePtr_t dev = createRobot ();
  BOOST_REQUIRE (dev);
  Problem problem (dev);

  ConstraintSetPtr_t c = createConstraints (dev);
  PolynomialPtr_t parabola (new Polynomial (dev));
  parabola->coefs_(1) = 0;
  c->configProjector ()->add (NumericalConstraint::create (parabola));
  problem.steeringMethod ()->constraints (c);

  pathProjector::ProgressivePtr_t progressive =
    pathProjector::Progressive::create (problem.distance(),
        problem.steeringMethod(), 0.1);

  Configuration_t q1 (dev->configSize()); q1 <<  1,0;
  Configuration_t q2 (dev->configSize()); q2 << -1,1;
  PathPtr_t path = (*problem.steeringMethod ()) (q1,q2);

  PathPtr_t projection;
  // Averaging the projection
  bool success;
  for (int i = 0; i < 100; ++i) {
    HPP_START_TIMECOUNTER (progressiveParabola);
    bool success = progressive->apply (path, projection);
    HPP_STOP_TIMECOUNTER (progressiveParabola);
    HPP_DISPLAY_LAST_TIMECOUNTER (progressiveParabola);
  }
  HPP_DISPLAY_TIMECOUNTER (progressiveParabola);
  std::cout << success << std::endl;
  displayPaths (path, projection, parabola);
}
