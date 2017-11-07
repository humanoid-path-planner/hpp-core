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

#define BOOST_TEST_MODULE spline_path
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/problem.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/pinocchio/simple-device.hh>

using namespace hpp::core;
using namespace hpp::pinocchio;

DevicePtr_t createRobot ()
{
  DevicePtr_t robot = unittest::makeDevice(unittest::ManipulatorArm2);
  return robot;
}

BOOST_AUTO_TEST_CASE (memory_deallocation)
{
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->robot(createRobot());
  
  ProblemPtr_t problem = ps->problem();
  DeviceWkPtr_t dev = ps->robot();
  SteeringMethodWkPtr_t sm = problem->steeringMethod();
  DistanceWkPtr_t distance = problem->distance ();
  RoadmapWkPtr_t roadmap = ps->roadmap ();

  delete ps;

  BOOST_CHECK_MESSAGE( !dev     .lock(), "Device was not deleted");
  BOOST_CHECK_MESSAGE( !sm      .lock(), "Steering method was not deleted");
  BOOST_CHECK_MESSAGE( !distance.lock(), "Distance was not deleted");
  BOOST_CHECK_MESSAGE( !roadmap .lock(), "Roadmap was not deleted");
}
