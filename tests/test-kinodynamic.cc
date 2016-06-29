// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of the hpp-core.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#include <sstream>
#include <ostream>
#include <fstream>
#include <vector>

# include <hpp/core/steering-method/steering-kinodynamic.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>
# include <hpp/core/kinodynamic-path.hh>

#include <boost/assign.hpp>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/roadmap.hh>
#include "hpp/core/basic-configuration-shooter.hh"
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/nearest-neighbor.hh>

#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>

#define BOOST_TEST_MODULE kinodynamic
#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace core;
using namespace std;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

BOOST_AUTO_TEST_CASE (kinodynamic) {
  
  DevicePtr_t robot = model::Device::create("robot");
  JointPtr_t xJoint = new model::JointTranslation <1> (fcl::Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new model::JointTranslation <1>
    (fcl::Transform3f());
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);

  robot->rootJoint (xJoint);
  xJoint->addChildJoint (yJoint);
  
  robot->setDimensionExtraConfigSpace(2);
  BOOST_CHECK_MESSAGE( robot->configSize() == 4 , "error during creation of the robot");
  BOOST_CHECK_MESSAGE( robot->extraConfigSpace().dimension() == 2,"error during creation of the robot");
  
  
  

  // Create steering method
  Problem p = Problem (robot);
  steeringMethod::KinodynamicPtr_t sm = steeringMethod::Kinodynamic::create (&p);

}
BOOST_AUTO_TEST_SUITE_END()
