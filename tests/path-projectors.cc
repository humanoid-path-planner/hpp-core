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

#define BOOST_TEST_MODULE pathProjector
#include <boost/test/included/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>

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
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/interpolated-path.hh>

#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/progressive.hh>

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
  std::cerr << std::fixed << std::showpos << std::setprecision (4);
  const char* sep = "\t| ";
  for (std::size_t i = 0; i < 100; ++i) {
    if (!(*path) (q, i * stepPath))
      std::cerr << "Could not project path at " << i*stepPath << "\n";
    if (!(*projection) (qq, i * stepProj))
      std::cerr << "Could not project projection at " << i*stepProj << "\n";
    (*func) (v1, q);
    (*func) (v2, qq);
    std::cerr << q.transpose () << sep << v1
      << sep << qq.transpose () << sep << v2 << "\n";
  }

  // Analyse projection
  if (!HPP_DYNAMIC_PTR_CAST (InterpolatedPath, projection)) {
    std::cout << "Path is not an InterpolatedPath\n";
    std::cerr
      << projection->timeRange().first << sep << projection->initial ().transpose() << '\n'
      << projection->timeRange().first + projection->timeRange().second << sep << projection->end ().transpose() << '\n';
    return;
  }
  InterpolatedPath& p = *(HPP_DYNAMIC_PTR_CAST (InterpolatedPath, projection));
  typedef InterpolatedPath::InterpolationPoints_t InterpolationPoints_t;
  const InterpolationPoints_t& points = p.interpolationPoints ();
  std::cout << "InterpolatedPath: " << points.size() << " interpolation points.\n";
  for (InterpolationPoints_t::const_iterator it = points.begin();
      it != points.end(); ++it) {
    std::cerr << it->first << sep << it->second.transpose() << '\n';
  }
}

struct traits_circle {
  static DifferentiableFunctionPtr_t func (DevicePtr_t dev) {
    return PolynomialPtr_t (new Polynomial (dev));
  }
  static void make_conf (ConfigurationOut_t q1, ConfigurationOut_t q2,
      const int index) {
    switch (index) {
      case 0: q1 << 0, 1; q2 <<  1, 0; break;
      case 1: q1 << 1, 0; q2 << -1, 0; break;
      case 2: {
                double c = -0.99;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
      case 3: {
                double c = -0.9;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
    }
  }
  static const int NB_CONFS;
  static const char* _func;
};
struct traits_parabola {
  static DifferentiableFunctionPtr_t func (DevicePtr_t dev) {
    PolynomialPtr_t parabola (new Polynomial (dev));
    parabola->coefs_(1) = 0;
    return parabola;
  }
  static void make_conf (ConfigurationOut_t q1, ConfigurationOut_t q2,
      const int index) {
    switch (index) {
      case 0: q1 <<  1,0; q2 << -1,0; break; // Should be really fast
      case 1: q1 <<  1,0; q2 << -1,1; break; // Should be slower
    }
  }
  static const int NB_CONFS;
  static const char* _func;
};
const int traits_circle::NB_CONFS = 4;
const char* traits_circle::_func = "circle";
const int traits_parabola::NB_CONFS = 2;
const char* traits_parabola::_func = "parabola";

struct traits_progressive {
  typedef pathProjector::Progressive Proj_t;
  typedef pathProjector::ProgressivePtr_t ProjPtr_t;
  static const value_type projection_step;
  static const char* _proj;
};
struct traits_global {
  typedef pathProjector::Global Proj_t;
  typedef pathProjector::GlobalPtr_t ProjPtr_t;
  static const value_type projection_step;
  static const char* _proj;
};
const value_type traits_progressive::projection_step = 0.1;
const value_type traits_global::projection_step = 0.1;
const char* traits_progressive::_proj = "progressive";
const char* traits_global::_proj = "global";

struct traits_global_circle : traits_global, traits_circle {};
struct traits_global_parabola : traits_global, traits_parabola {};
struct traits_progressive_circle : traits_progressive, traits_circle {};
struct traits_progressive_parabola : traits_progressive, traits_parabola {};

typedef boost::mpl::list <traits_global_circle,
                          traits_progressive_circle,
                          traits_global_parabola,
                          traits_progressive_parabola
                          > test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE (projectors, traits, test_types)
{
  DevicePtr_t dev = createRobot ();
  BOOST_REQUIRE (dev);
  Problem problem (dev);

  ConstraintSetPtr_t c = createConstraints (dev);
  DifferentiableFunctionPtr_t func = traits::func (dev);
  c->configProjector ()->add (NumericalConstraint::create (func));
  problem.steeringMethod ()->constraints (c);

  typename traits::ProjPtr_t projector =
    traits::Proj_t::create (problem.distance(), problem.steeringMethod(),
        traits::projection_step);

  std::cout << "========================================\n";

  Configuration_t q1 (dev->configSize());
  Configuration_t q2 (dev->configSize());
  for (int i = 0; i < traits::NB_CONFS; ++i) {
    HPP_DEFINE_TIMECOUNTER(projector);
    traits::make_conf (q1, q2, i);
    PathPtr_t path = (*problem.steeringMethod ()) (q1,q2);

    PathPtr_t projection;
    // Averaging the projection
    bool success;
    for (int j = 0; j < 1; ++j) {
      HPP_START_TIMECOUNTER (projector);
      success = projector->apply (path, projection);
      HPP_STOP_TIMECOUNTER (projector);
      HPP_DISPLAY_LAST_TIMECOUNTER (projector);
    }
    std::cout << traits::_proj << " " << traits::_func << ": projection of "
      << q1.transpose() << " -> " << q2.transpose() << " "
      << (success?"succeeded.":"failed.") << std::endl;
    HPP_STREAM_TIMECOUNTER (std::cout, projector) << std::endl;
    displayPaths (path, projection, func);
  }
}
