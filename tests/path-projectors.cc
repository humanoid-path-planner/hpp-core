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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>

#include <hpp/core/straight-path.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path/hermite.hh>

#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/steering-method/hermite.hh>

#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/progressive.hh>

#include <hpp/core/path-projector/recursive-hermite.hh>

using hpp::constraints::Implicit;

using namespace hpp::core;
using namespace hpp::pinocchio;

DevicePtr_t createRobot ()
{
  std::string urdf ("<robot name='test'>"
      "<link name='link1'/>"
      "<link name='link2'/>"
      "<link name='link3'/>"
      "<joint name='tx' type='prismatic'>"
        "<parent link='link1'/>"
        "<child  link='link2'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "<joint name='ty' type='prismatic'>"
        "<axis xyz='0 1 0'/>"
        "<parent link='link2'/>"
        "<child  link='link3'/>"
        "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
      "</joint>"
      "</robot>"
      );

  DevicePtr_t robot = Device::create ("test");
  urdf::loadModelFromString (robot, 0, "", "anchor", urdf, "");
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
      DifferentiableFunction (robot->configSize(), robot->numberDof (),
                              LiegroupSpace::R1 (), "Polynomial"),
      coefs_ (vector_t::Ones (robot->configSize()))
    {}

    vector_t coefs_;

  protected:
      void impl_compute (LiegroupElementRef result, vectorIn_t argument) const {
        result.vector ()[0] = argument.cwiseProduct (argument).dot (coefs_) - 1;
      }
      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const {
        jacobian.row(0) = 2 * arg.cwiseProduct (coefs_);
      }
};

typedef boost::shared_ptr<Polynomial> PolynomialPtr_t;

/*
bool checkContinuity (PathPtr_t path) {
  const value_type stepPath = path->length () / (100 - 1);
  Configuration_t q = path->initial(), qq = path->initial();
  vector_t v1 (func->outputSize()), v2(func->outputSize());
  std::cerr << std::fixed << std::showpos << std::setprecision (4);
  const char* sep = "\t| ";
  for (std::size_t i = 0; i < 100; ++i) {
    if (!(*path) (q, (value_type)i * stepPath))
      std::cerr << "Could not project path at " << (value_type)i*stepPath
		<< "\n";
    if (!(*projection) (qq, (value_type) i * stepProj))
      std::cerr << "Could not project projection at "
		<< (value_type) i*stepProj << "\n";
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
// */
void displayPaths (PathPtr_t path, PathPtr_t projection, DifferentiableFunctionPtr_t func) {
  const value_type stepPath = path->length () / (100 - 1);
  const value_type stepProj = projection->length () / (100 - 1);
  Configuration_t q = path->initial(), qq = path->initial();
  LiegroupElement v1 (func->outputSpace ()), v2(func->outputSpace());
  std::cerr << std::fixed << std::showpos << std::setprecision (4);
  const char* sep = "\t| ";
  for (std::size_t i = 0; i < 100; ++i) {
    if (!(*path) (q, (value_type)i * stepPath))
      std::cerr << "Could not project path at " << (value_type)i*stepPath
		<< "\n";
    if (!(*projection) (qq, (value_type) i * stepProj))
      std::cerr << "Could not project projection at "
		<< (value_type) i*stepProj << "\n";
    func->value (v1, q);
    func->value (v2, qq);
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
      case 4: {
                double c = -0.85;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
      case 5: {
                double c = -0.8;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
      case 6: {
                double c = -0.75;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
      case 7: {
                double c = -0.7;
                q1 << 1, 0; q2 << c, sqrt(1 - c*c);
                break;
              }
    }
  }
  static const int NB_CONFS;
  static const value_type K;
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
    q1 <<  1,0; q2 << -1,(value_type)(2*index) / (NB_CONFS - 1);
    // switch (index) {
      // case 0: q1 <<  1,0; q2 << -1,0; break; // Should be really fast
      // case 1: q1 <<  1,0; q2 << -1,1; break; // Should be slower
    // }
  }
  static const int NB_CONFS;
  static const value_type K;
  static const char* _func;
};
const int traits_circle::NB_CONFS = 8;
const value_type traits_circle::K = 2.001;
const char* traits_circle::_func = "circle";
const int traits_parabola::NB_CONFS = 8;
const value_type traits_parabola::K = 2 * sqrt(2);
const char* traits_parabola::_func = "parabola";

struct traits_progressive {
  typedef pathProjector::Progressive Proj_t;
  typedef pathProjector::ProgressivePtr_t ProjPtr_t;
  typedef steeringMethod::Straight SM_t;
  static const value_type projection_step;
  static const char* _proj;
};
struct traits_global {
  typedef pathProjector::Global Proj_t;
  typedef pathProjector::GlobalPtr_t ProjPtr_t;
  typedef steeringMethod::Straight SM_t;
  static const value_type projection_step;
  static const char* _proj;
};
struct traits_hermite {
  typedef pathProjector::RecursiveHermite Proj_t;
  typedef pathProjector::RecursiveHermitePtr_t ProjPtr_t;
  typedef steeringMethod::Hermite SM_t;
  static const value_type projection_step;
  static const char* _proj;
};
const value_type traits_progressive::projection_step = 0.1;
const value_type traits_global     ::projection_step = 0.1;
const value_type traits_hermite    ::projection_step = 2;
const char* traits_progressive::_proj = "progressive";
const char* traits_global     ::_proj = "global";
const char* traits_hermite    ::_proj = "hermite";

struct traits_global_circle        : traits_global     , traits_circle   {};
struct traits_global_parabola      : traits_global     , traits_parabola {};
struct traits_progressive_circle   : traits_progressive, traits_circle   {};
struct traits_progressive_parabola : traits_progressive, traits_parabola {};
struct traits_hermite_circle       : traits_hermite    , traits_circle   {};
struct traits_hermite_parabola     : traits_hermite    , traits_parabola {};

typedef boost::mpl::list <  traits_global_circle
                          , traits_progressive_circle
                          , traits_hermite_circle
                          , traits_global_parabola
                          , traits_progressive_parabola
                          , traits_hermite_parabola
                          > test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE (projectors, traits, test_types)
{
  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE (dev);
  ProblemPtr_t problem = Problem::create(dev);

  ConstraintSetPtr_t c = createConstraints (dev);
  DifferentiableFunctionPtr_t func = traits::func (dev);
  c->configProjector ()->add (Implicit::create (func));
  problem->steeringMethod(traits::SM_t::create (*problem));
  problem->steeringMethod ()->constraints (c);

  for (int c = 0; c < 2; ++c) {
    if (c == 0)
      problem->setParameter ("PathProjection/HessianBound", Parameter((value_type)-1));
    else
      problem->setParameter ("PathProjection/HessianBound", Parameter(traits::K));

    typename traits::ProjPtr_t projector =
      traits::Proj_t::create (*problem, traits::projection_step);

    std::cout << "========================================\n";

    Configuration_t q1 (dev->configSize());
    Configuration_t q2 (dev->configSize());
    for (int i = 0; i < traits::NB_CONFS; ++i) {

      // HPP_DEFINE_TIMECOUNTER(projector);
      traits::make_conf (q1, q2, i);
      PathPtr_t path = (*problem->steeringMethod ()) (q1,q2);

      PathPtr_t projection;
      // Averaging the projection
      bool success;
      for (int j = 0; j < 2; ++j) {
        // HPP_START_TIMECOUNTER (projector);
        success = projector->apply (path, projection);
        // HPP_STOP_TIMECOUNTER (projector);
        // HPP_DISPLAY_LAST_TIMECOUNTER (projector);
      }
      std::cout << traits::_proj << " " << traits::_func << ": projection of "
        << q1.transpose() << " -> " << q2.transpose() << " "
        << (success?"succeeded.":"failed.") << std::endl;
      // HPP_STREAM_TIMECOUNTER (std::cout, projector) << std::endl;
      // displayPaths (path, projection, func);
    }
  }
}
