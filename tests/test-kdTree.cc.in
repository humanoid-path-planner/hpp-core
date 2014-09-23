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
#include <hpp/model/joint.hh>
#include <hpp/core/fwd.hh>
//#include <hpp/model/fwd.hh>
#include <hpp/core/roadmap.hh>
#include "../src/nearest-neighbor.hh"
#include <hpp/core/k-d-tree.hh>
#include <hpp/core/weighed-distance.hh>
#include "hpp/core/basic-configuration-shooter.hh"
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include "../src/node.cc"
#include "../src/k-d-tree.cc"
#include "../src/weighed-distance.cc"
#include <hpp/model/joint-configuration.hh>


#define BOOST_TEST_MODULE kdTree
#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace core;
using namespace model;
using namespace std;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

BOOST_AUTO_TEST_CASE (kdTree) {
  // Build Device
  DevicePtr_t robot = Device::create("robot");
  JointPtr_t xJoint = new JointTranslation <1> (Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new JointTranslation <1> (Transform3f());
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);
  JointPtr_t so3Joint = new JointSO3(Transform3f());
  robot->rootJoint(so3Joint);
  robot->registerJoint(xJoint);
  robot->registerJoint(yJoint);

  // Build Distance, nearestNeighbor, KDTree
  WeighedDistancePtr_t weighedDistance = WeighedDistance::create(robot);
  for ( int i =0 ; i<3 ; i++ ) weighedDistance->setWeight(i,1);
  DistancePtr_t distance = weighedDistance;
  BasicConfigurationShooter confShoot(robot);
  KDTree kdTree(robot,distance,30);
  typedef std::map <ConnectedComponentPtr_t, NearestNeighborPtr_t>
    NearetNeighborMap_t;
  NearetNeighborMap_t nearestNeighbor;

  // Add 4 connectedComponents with 2000 nodes each
  ConfigurationPtr_t configuration;
  NodePtr_t node;
  ConnectedComponentPtr_t connectedComponent[4];
  for ( int i=0 ; i<4 ; i++ ) {
    connectedComponent[i] = ConnectedComponent::create();
    nearestNeighbor[connectedComponent[i]] =
      NearestNeighborPtr_t (new NearestNeighbor (distance));
    for ( int j=0 ; j<200 ; j++ ) {
      configuration = confShoot.shoot();
      node = NodePtr_t(new Node(configuration, connectedComponent[i]));
      nearestNeighbor[ (connectedComponent[i]) ]->add(node);
      kdTree.addNode(node);
    }
  }

  // search nearest node
  value_type minDistance1;
  value_type minDistance2;
  NodePtr_t node1;
  NodePtr_t node2;
  for ( int j=0 ; j<200 ; j++ ) {
    configuration = confShoot.shoot();
    for ( int i=0 ; i<4 ; i++ ) {
      minDistance1 = std::numeric_limits<value_type>::infinity ();
      minDistance2 = std::numeric_limits<value_type>::infinity ();
      node1 = nearestNeighbor[connectedComponent[i]]
	->nearest(configuration,minDistance1);
      node2 = kdTree.search(configuration, connectedComponent[i], minDistance2);
      BOOST_CHECK( node1 == node2 );
      BOOST_CHECK( minDistance1 == minDistance2 );
    }
  }
}
BOOST_AUTO_TEST_SUITE_END()



