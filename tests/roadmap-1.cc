// Copyright (C) 2014 LAAS-CNRS
// Author: Mathieu Geisert
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

#include <boost/assign.hpp>
#include <hpp/core/connected-component.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/node.hh>
#include <hpp/core/parser/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/util/debug.hh>

#define BOOST_TEST_MODULE roadmap - 1
#include <boost/test/included/unit_test.hpp>

using namespace hpp::core;
using namespace hpp::pinocchio;

using hpp::core::steeringMethod::Straight;
using hpp::core::steeringMethod::StraightPtr_t;

BOOST_AUTO_TEST_SUITE(test_hpp_core)

void addEdge(const hpp::core::RoadmapPtr_t& r,
             const hpp::core::SteeringMethod& sm,
             const std::vector<NodePtr_t>& nodes, std::size_t i,
             std::size_t j) {
  r->addEdge(nodes[i], nodes[j],
             sm(nodes[i]->configuration(), nodes[j]->configuration()));
}

DevicePtr_t createRobot() {
  std::string urdf(
      "<robot name='test'>"
      "<link name='link1'/>"
      "<link name='link2'/>"
      "<link name='link3'/>"
      "<joint name='tx' type='prismatic'>"
      "<parent link='link1'/>"
      "<child  link='link2'/>"
      "<limit effort='30' velocity='1.0' lower='-3' upper='3'/>"
      "</joint>"
      "<joint name='ty' type='prismatic'>"
      "<axis xyz='0 1 0'/>"
      "<parent link='link2'/>"
      "<child  link='link3'/>"
      "<limit effort='30' velocity='1.0' lower='-3' upper='3'/>"
      "</joint>"
      "</robot>");

  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "", "anchor", urdf, "");
  return robot;
}

BOOST_AUTO_TEST_CASE(Roadmap1) {
  // Build robot
  DevicePtr_t robot = createRobot();

  // Create steering method
  ProblemPtr_t p = Problem::create(robot);
  StraightPtr_t sm = Straight::create(p);
  // create roadmap
  hpp::core::DistancePtr_t distance(
      WeighedDistance::createWithWeight(robot, vector_t::Ones(2)));
  RoadmapPtr_t r = Roadmap::create(distance, robot);

  std::vector<NodePtr_t> nodes;

  // nodes [0]
  Configuration_t q(robot->configSize());
  q[0] = 0;
  q[1] = 0;
  // Set init node
  r->initNode(q);
  nodes.push_back(r->initNode());

  // nodes [1]
  q = Configuration_t(robot->configSize());
  q[0] = 1;
  q[1] = 0;
  nodes.push_back(r->addNode(q));

  // nodes [2]
  q = Configuration_t(robot->configSize());
  q[0] = 0.5;
  q[1] = 0.9;
  nodes.push_back(r->addNode(q));

  // nodes [3]
  q = Configuration_t(robot->configSize());
  q[0] = -0.1;
  q[1] = -0.9;
  nodes.push_back(r->addNode(q));

  // nodes [4]
  q = Configuration_t(robot->configSize());
  q[0] = 1.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));

  // nodes [5]
  q = Configuration_t(robot->configSize());
  q[0] = 2.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));
  r->addGoalNode(nodes[5]->configuration());

  BOOST_TEST_MESSAGE(*r);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 6);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  // 0 -> 1
  addEdge(r, *sm, nodes, 0, 1);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 6);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 1 -> 0
  addEdge(r, *sm, nodes, 1, 0);

  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 5);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);
  // 1 -> 2
  addEdge(r, *sm, nodes, 1, 2);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 5);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 2 -> 0
  addEdge(r, *sm, nodes, 2, 0);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 4);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  for (std::size_t i = 2; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 2 -> 3
  addEdge(r, *sm, nodes, 2, 3);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 4);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  for (std::size_t i = 2; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 2 -> 4
  addEdge(r, *sm, nodes, 2, 4);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 4);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  for (std::size_t i = 2; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(!r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 3 -> 5
  addEdge(r, *sm, nodes, 3, 5);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 4);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  for (std::size_t i = 2; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 4 -> 5
  addEdge(r, *sm, nodes, 4, 5);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 4);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  for (std::size_t i = 2; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      BOOST_CHECK_MESSAGE(
          nodes[i]->connectedComponent() != nodes[j]->connectedComponent(),
          "nodes " << i << " and " << j << "are in the same"
                   << " connected component");
    }
  }
  BOOST_CHECK(r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // 5 -> 0
  addEdge(r, *sm, nodes, 5, 0);
  BOOST_CHECK_EQUAL(r->connectedComponents().size(), 1);
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[1]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[2]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[3]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[4]->connectedComponent());
  BOOST_CHECK(nodes[0]->connectedComponent() == nodes[5]->connectedComponent());
  BOOST_CHECK(r->pathExists());
  BOOST_TEST_MESSAGE(*r);

  // Check that memory if well deallocated.
  std::list<ConnectedComponentWkPtr_t> ccs;
  for (ConnectedComponents_t::const_iterator _cc =
           r->connectedComponents().begin();
       _cc != r->connectedComponents().end(); ++_cc)
    ccs.push_back(*_cc);

  r.reset();
  for (std::list<ConnectedComponentWkPtr_t>::const_iterator _cc = ccs.begin();
       _cc != ccs.begin(); ++_cc) {
    BOOST_CHECK(!_cc->lock());
  }
}

BOOST_AUTO_TEST_CASE(nearestNeighbor) {
  // Build robot
  DevicePtr_t robot = createRobot();

  // Create steering method
  ProblemPtr_t p = Problem::create(robot);
  StraightPtr_t sm = Straight::create(p);
  // create roadmap
  hpp::core::DistancePtr_t distance(
      WeighedDistance::createWithWeight(robot, vector_t::Ones(2)));
  RoadmapPtr_t r = Roadmap::create(distance, robot);

  std::vector<NodePtr_t> nodes;

  // nodes [0]
  Configuration_t q(robot->configSize());
  q[0] = 0;
  q[1] = 0;
  nodes.push_back(r->addNode(q));

  // nodes [1]
  q = Configuration_t(robot->configSize());
  q[0] = 1;
  q[1] = 0;
  nodes.push_back(r->addNode(q));

  // nodes [2]
  q = Configuration_t(robot->configSize());
  q[0] = 0.5;
  q[1] = 0.9;
  nodes.push_back(r->addNode(q));

  // nodes [3]
  q = Configuration_t(robot->configSize());
  q[0] = -0.1;
  q[1] = -0.9;
  nodes.push_back(r->addNode(q));

  // nodes [4]
  q = Configuration_t(robot->configSize());
  q[0] = 1.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));

  // nodes [5]
  q = Configuration_t(robot->configSize());
  q[0] = 2.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));
  r->addGoalNode(nodes[5]->configuration());

  // nodes [6]
  q = Configuration_t(robot->configSize());
  q[0] = 0;
  q[1] = 0.2;
  nodes.push_back(r->addNode(q));
  r->addGoalNode(nodes[6]->configuration());

  // 0 -> 1
  addEdge(r, *sm, nodes, 0, 1);
  // 1 -> 0
  addEdge(r, *sm, nodes, 1, 0);
  // 1 -> 2
  addEdge(r, *sm, nodes, 1, 2);
  // 2 -> 0
  addEdge(r, *sm, nodes, 2, 0);
  // 0 -> 3
  addEdge(r, *sm, nodes, 0, 3);
  // 3 -> 2
  addEdge(r, *sm, nodes, 3, 2);

  hpp::pinocchio::value_type dist;
  using hpp::core::Nodes_t;
  Nodes_t knearest = r->nearestNeighbor()->KnearestSearch(
      nodes[0]->configuration(), nodes[0]->connectedComponent(), 3, dist);
  for (Nodes_t::const_iterator it = knearest.begin(); it != knearest.end();
       ++it) {
    BOOST_TEST_MESSAGE("q = [" << (*it)->configuration().transpose()
                               << "] - dist : "
                               << (*distance)(nodes[0]->configuration(),
                                              (*it)->configuration()));
  }
  for (std::vector<NodePtr_t>::const_iterator it = nodes.begin();
       it != nodes.end(); ++it) {
    Configuration_t q = (*it)->configuration();
    BOOST_TEST_MESSAGE("[" << q.transpose() << "] - dist : "
                           << (*distance)(nodes[0]->configuration(), q));
  }
}

BOOST_AUTO_TEST_CASE(serialization) {
  // Build robot
  DevicePtr_t robot = createRobot();

  // Create steering method
  ProblemPtr_t p = Problem::create(robot);
  StraightPtr_t sm = Straight::create(p);
  // create roadmap
  hpp::core::DistancePtr_t distance(
      WeighedDistance::createWithWeight(robot, vector_t::Ones(2)));
  RoadmapPtr_t r = Roadmap::create(distance, robot);

  std::vector<NodePtr_t> nodes;

  // nodes [0]
  Configuration_t q(robot->configSize());
  q[0] = 0;
  q[1] = 0;
  // Set init node
  r->initNode(q);
  nodes.push_back(r->initNode());

  // nodes [1]
  q = Configuration_t(robot->configSize());
  q[0] = 1;
  q[1] = 0;
  nodes.push_back(r->addNode(q));

  // nodes [2]
  q = Configuration_t(robot->configSize());
  q[0] = 0.5;
  q[1] = 0.9;
  nodes.push_back(r->addNode(q));

  // nodes [3]
  q = Configuration_t(robot->configSize());
  q[0] = -0.1;
  q[1] = -0.9;
  nodes.push_back(r->addNode(q));

  // nodes [4]
  q = Configuration_t(robot->configSize());
  q[0] = 1.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));

  // nodes [5]
  q = Configuration_t(robot->configSize());
  q[0] = 2.5;
  q[1] = 2.9;
  nodes.push_back(r->addNode(q));
  r->addGoalNode(nodes[5]->configuration());

  // 1 -> 0
  addEdge(r, *sm, nodes, 0, 1);
  // 1 -> 2
  addEdge(r, *sm, nodes, 1, 2);
  // 2 -> 0
  addEdge(r, *sm, nodes, 2, 0);
  // 3 -> 5
  addEdge(r, *sm, nodes, 3, 5);
  // 4 -> 5
  addEdge(r, *sm, nodes, 4, 5);

  std::cout << *r << std::endl;

  // save data to archive
  parser::serializeRoadmap<hpp::serialization::text_oarchive>(
      r, "filename.txt", parser::make_nvp(robot->name(), robot.get()));

  parser::serializeRoadmap<hpp::serialization::xml_oarchive>(
      r, "filename.xml", parser::make_nvp(robot->name(), robot.get()));

  parser::serializeRoadmap<hpp::serialization::binary_oarchive>(
      r, "filename.bin", parser::make_nvp(robot->name(), robot.get()));
}

BOOST_AUTO_TEST_CASE(deserialization) {
  DevicePtr_t robot = createRobot();

  RoadmapPtr_t nr;
  parser::serializeRoadmap<hpp::serialization::text_iarchive>(
      nr, "filename.txt", parser::make_nvp(robot->name(), robot.get()));
  std::cout << *nr << std::endl;

  parser::serializeRoadmap<hpp::serialization::xml_iarchive>(
      nr, "filename.xml", parser::make_nvp(robot->name(), robot.get()));
  std::cout << *nr << std::endl;

  parser::serializeRoadmap<hpp::serialization::binary_iarchive>(
      nr, "filename.bin", parser::make_nvp(robot->name(), robot.get()));
  std::cout << *nr << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
