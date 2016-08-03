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

#include <boost/assign.hpp>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/weighed-distance.hh>
#include "hpp/core/basic-configuration-shooter.hh"
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/model/joint-configuration.hh>

#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>

#define BOOST_TEST_MODULE roadmap-1
#include <boost/test/included/unit_test.hpp>

using hpp::model::Configuration_t;
using hpp::core::ConfigurationPtr_t;
using hpp::model::JointPtr_t;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointTranslation;
using hpp::core::Problem;
using hpp::core::SteeringMethodStraight;
using hpp::core::SteeringMethodStraightPtr_t;
using hpp::core::RoadmapPtr_t;
using hpp::core::Roadmap;
using hpp::core::NodePtr_t;
using hpp::core::WeighedDistance;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

void addEdge (const hpp::core::RoadmapPtr_t& r,
	      const hpp::core::SteeringMethod& sm,
	      const std::vector <NodePtr_t>& nodes,
	      std::size_t i, std::size_t j)
{
  r->addEdge (nodes [i], nodes [j], sm (*(nodes [i]->configuration ()),
					*(nodes [j]->configuration ())));
}

BOOST_AUTO_TEST_CASE (Roadmap1) {
  // Build robot
  DevicePtr_t robot = Device::create("robot");
  JointPtr_t xJoint = new JointTranslation <1> (fcl::Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new JointTranslation <1>
    (fcl::Transform3f(fcl::Quaternion3f (sqrt (2)/2, 0, 0, sqrt(2)/2)));
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);

  robot->rootJoint (xJoint);
  xJoint->addChildJoint (yJoint);

  // Create steering method
  Problem p = Problem (robot);
  SteeringMethodStraightPtr_t sm = SteeringMethodStraight::create (&p);
  // create roadmap
  hpp::core::DistancePtr_t distance (WeighedDistance::createWithWeight
				     (robot, boost::assign::list_of (1)(1)));
  RoadmapPtr_t r = Roadmap::create (distance, robot);

  std::vector <NodePtr_t> nodes;

  // nodes [0]
  ConfigurationPtr_t q (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0;
  // Set init node
  r->initNode (q);
  nodes.push_back (r->initNode ());

  // nodes [1]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [2]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0.5; (*q) [1] = 0.9;
  nodes.push_back (r->addNode (q));

  // nodes [3]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = -0.1; (*q) [1] = -0.9;
  nodes.push_back (r->addNode (q));

  // nodes [4]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));

  // nodes [5]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 2.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [5]->configuration ());

  std::cout << *r << std::endl;
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 6);
  for (std::size_t i=0; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  // 0 -> 1
  addEdge (r, *sm, nodes, 0, 1);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 6);
  for (std::size_t i=0; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 1 -> 0
  addEdge (r, *sm, nodes, 1, 0);

  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 5);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  for (std::size_t i=1; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl; 
  // 1 -> 2
  addEdge (r, *sm, nodes, 1, 2);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 5);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  for (std::size_t i=1; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 0
  addEdge (r, *sm, nodes, 2, 0);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 3
  addEdge (r, *sm, nodes, 2, 3);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 4
  addEdge (r, *sm, nodes, 2, 4);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 3 -> 5
  addEdge (r, *sm, nodes, 3, 5);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

  // 4 -> 5
  addEdge (r, *sm, nodes, 4, 5);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

  // 5 -> 0
  addEdge (r, *sm, nodes, 5, 0);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 1);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [3]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [4]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [5]->connectedComponent ());
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

}

BOOST_AUTO_TEST_CASE (nearestNeighbor) {
  // Build robot
  DevicePtr_t robot = Device::create("robot");
  JointPtr_t xJoint = new JointTranslation <1> (fcl::Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new JointTranslation <1>
    (fcl::Transform3f(fcl::Quaternion3f (sqrt (2)/2, 0, 0, sqrt(2)/2)));
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);

  robot->rootJoint (xJoint);
  xJoint->addChildJoint (yJoint);

  // Create steering method
  Problem p (robot);
  SteeringMethodStraightPtr_t sm = SteeringMethodStraight::create (&p);
  // create roadmap
  hpp::core::DistancePtr_t distance (WeighedDistance::createWithWeight
				     (robot, boost::assign::list_of (1)(1)));
  RoadmapPtr_t r = Roadmap::create (distance, robot);

  std::vector <NodePtr_t> nodes;

  // nodes [0]
  ConfigurationPtr_t q (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [1]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [2]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0.5; (*q) [1] = 0.9;
  nodes.push_back (r->addNode (q));

  // nodes [3]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = -0.1; (*q) [1] = -0.9;
  nodes.push_back (r->addNode (q));

  // nodes [4]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));

  // nodes [5]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 2.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [5]->configuration ());

  // nodes [6]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0.2;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [6]->configuration ());

  // 0 -> 1
  addEdge (r, *sm, nodes, 0, 1);
  // 1 -> 0
  addEdge (r, *sm, nodes, 1, 0);
  // 1 -> 2
  addEdge (r, *sm, nodes, 1, 2);
  // 2 -> 0
  addEdge (r, *sm, nodes, 2, 0);
  // 0 -> 3
  addEdge (r, *sm, nodes, 0, 3);
  // 3 -> 2
  addEdge (r, *sm, nodes, 3, 2);

  hpp::model::value_type dist;
  using hpp::core::Nodes_t;
  Nodes_t knearest = r->nearestNeighbor()->KnearestSearch
    (nodes[0]->configuration(), nodes[0]->connectedComponent (), 3, dist);
  for (Nodes_t::const_iterator it = knearest.begin (); it != knearest.end(); ++it) {
    BOOST_MESSAGE ("q = [" << (*it)->configuration()->transpose() << "] - dist : " << (*distance) (*nodes[0]->configuration(), *(*it)->configuration()));
  }
  for (std::vector<NodePtr_t>::const_iterator it = nodes.begin (); it != nodes.end(); ++it) {
    Configuration_t& q = *(*it)->configuration();
    BOOST_MESSAGE ("[" << q.transpose() << "] - dist : " << (*distance) (*nodes[0]->configuration(), q));
  }
}
BOOST_AUTO_TEST_SUITE_END()



