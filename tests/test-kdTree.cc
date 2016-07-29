// Copyright (C) 2014 LAAS-CNRS
// Author: Mathieu Geisert
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

//#include <Eigen/Core>

#include <hpp/util/debug.hh>

#include <hpp/model/device.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/weighed-distance.hh>
#include "hpp/core/basic-configuration-shooter.hh"
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/steering-method-straight.hh>
#include "../src/nearest-neighbor/basic.hh"
#include "../src/nearest-neighbor/k-d-tree.hh"


#define BOOST_TEST_MODULE kdTree
#include <boost/test/included/unit_test.hpp>
#include <pinocchio/multibody/joint/joint-variant.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include "../tests/utils.hh"


using namespace hpp;
using namespace core;
using namespace pinocchio;
using namespace std;
using ::se3::JointModelPX;
using ::se3::JointModelPY;
using ::se3::JointModelPZ;
using ::se3::JointIndex;
using ::se3::JointModelSpherical;
using ::se3::JointModelRUBZ;


BOOST_AUTO_TEST_SUITE( test_hpp_core )

BOOST_AUTO_TEST_CASE (kdTree) {
  // Build Device
 /* DevicePtr_t robot = Device::create("robot");
  JointPtr_t transJoint = new JointTranslation <3> (Transform3f());
  transJoint->isBounded (0, true);
  transJoint->lowerBound(0,-3.);
  transJoint->upperBound(0, 3.);
  transJoint->isBounded (1, true);
  transJoint->lowerBound(1,-3.);
  transJoint->upperBound(1, 3.);
  transJoint->isBounded (2, true);
  transJoint->lowerBound(2,-3.);
  transJoint->upperBound(2, 3.);
  JointPtr_t so3Joint = new JointSO3(Transform3f());
  JointPtr_t so2Joint = new jointRotation::UnBounded
    (Transform3f (fcl::Vec3f (0, 0, 1)));
  robot->rootJoint(transJoint);
  transJoint->addChildJoint (so3Joint);
  so3Joint->addChildJoint (so2Joint);
*/
  DevicePtr_t robot = Device::create("robot");
  ModelPtr_t m = ModelPtr_t(new ::se3::Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new ::se3::GeometryModel());
  robot->model(m);
  robot->geomModel(gm);
  const std::string& name = robot->name ();
  Transform3f mat; mat.setIdentity ();

  JointModelPX::TangentVector_t max_effort = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::TangentVector_t max_velocity = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::ConfigVector_t lower_position = JointModelPY::ConfigVector_t::Constant(-3.);
  JointModelPX::ConfigVector_t upper_position = JointModelPY::ConfigVector_t::Constant(3.);


  JointIndex idJoint = robot->model().addJoint(0,JointModelPX(), mat,name + "_x",max_effort,max_velocity,lower_position,upper_position);
  idJoint = robot->model().addJoint(idJoint,JointModelPY(), mat,name + "_y",max_effort,max_velocity,lower_position,upper_position);
  idJoint = robot->model().addJoint(idJoint,JointModelPZ(), mat,name + "_z",max_effort,max_velocity,lower_position,upper_position);
  idJoint = robot->model().addJoint(idJoint,JointModelSpherical(), mat,name + "_SO3");
  idJoint = robot->model().addJoint(idJoint,JointModelRUBZ(), mat,name + "_SO2");

  robot->createData();
  robot->createGeomData();


  // Build Distance, nearestNeighbor, KDTree
  Problem problem (robot);
  WeighedDistancePtr_t distance = WeighedDistance::create(robot);
  problem.distance (distance);
  ConfigurationShooterPtr_t confShoot = problem.configurationShooter();
  nearestNeighbor::KDTree kdTree(robot,distance,30);
  nearestNeighbor::Basic basic (distance);
  SteeringMethodPtr_t sm = SteeringMethodStraight::create (&problem);

  // Add 4 connectedComponents with 2000 nodes each
  ConfigurationPtr_t configuration;
  NodePtr_t node;
  NodePtr_t rootNode [4];
  RoadmapPtr_t roadmap = Roadmap::create (distance, robot);
  roadmap->nearestNeighbor(&kdTree);
  for ( int i=0 ; i<4 ; i++ ) {
    configuration = confShoot->shoot();
    rootNode [i] = roadmap->addNode (configuration);
    for (int j=1 ; j<200 ; j++) {
      configuration = confShoot->shoot();
      PathPtr_t path = (*sm) (*(rootNode [i]->configuration ()),
			      *configuration);
      node = roadmap->addNodeAndEdges (rootNode [i], configuration, path);
      basic.addNode (node);
    }
  }

  // search nearest node
  value_type minDistance1;
  value_type minDistance2;
  NodePtr_t node1;
  NodePtr_t node2;
  for ( int j=0 ; j<200 ; j++ ) {
    configuration = confShoot->shoot();
    for ( int i=0 ; i<4 ; i++ ) {
      minDistance1 = std::numeric_limits<value_type>::infinity ();
      minDistance2 = std::numeric_limits<value_type>::infinity ();
      node1 = basic.search (configuration, rootNode [i]->connectedComponent (),
			    minDistance1);
      node2 = roadmap->nearestNode (configuration,
				    rootNode [i]->connectedComponent (),
				    minDistance2);
      BOOST_CHECK( node1 == node2 );
      BOOST_CHECK( minDistance1 == minDistance2 );
      std::cout << displayConfig (*(node1->configuration ())) << std::endl;
      std::cout << minDistance1 << std::endl;
    }
  }
}
BOOST_AUTO_TEST_SUITE_END()



