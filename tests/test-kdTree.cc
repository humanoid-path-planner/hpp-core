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

#include <fstream>
#include <ostream>
#include <sstream>
#include <vector>

// #include <Eigen/Core>

#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/debug.hh>

#include "../src/nearest-neighbor/basic.hh"
#include "../src/nearest-neighbor/k-d-tree.hh"

#define BOOST_TEST_MODULE kdTree
#include <boost/test/included/unit_test.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

using hpp::pinocchio::value_type;
using ::pinocchio::JointIndex;
using ::pinocchio::JointModelRUBZ;
using ::pinocchio::JointModelSpherical;
using ::pinocchio::JointModelTranslation;

using hpp::core::ConfigurationShooterPtr_t;
using hpp::core::NodePtr_t;
using hpp::core::PathPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::Roadmap;
using hpp::core::RoadmapPtr_t;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::WeighedDistance;
using hpp::core::WeighedDistancePtr_t;
using hpp::core::nearestNeighbor::KDTree;
using hpp::core::nearestNeighbor::KDTreePtr_t;

using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::displayConfig;
using hpp::pinocchio::GeomModel;
using hpp::pinocchio::GeomModelPtr_t;
using hpp::pinocchio::Model;
using hpp::pinocchio::ModelPtr_t;
using hpp::pinocchio::Transform3s;

BOOST_AUTO_TEST_SUITE(test_hpp_core)

BOOST_AUTO_TEST_CASE(kdTree) {
  // Build Device
  DevicePtr_t robot = Device::create("robot");
  ModelPtr_t m = ModelPtr_t(new Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new hpp::pinocchio::GeomModel());
  robot->setModel(m);
  robot->setGeomModel(gm);
  const std::string& name = robot->name();
  Transform3s mat;
  mat.setIdentity();

  JointModelTranslation::TangentVector_t max_effort_tr =
      JointModelTranslation::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelTranslation::TangentVector_t max_velocity_tr =
      JointModelTranslation::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelTranslation::ConfigVector_t lower_position_tr =
      JointModelTranslation::ConfigVector_t::Constant(-3.);
  JointModelTranslation::ConfigVector_t upper_position_tr =
      JointModelTranslation::ConfigVector_t::Constant(3.);

  JointModelSpherical::TangentVector_t max_effort_SO3 =
      JointModelSpherical::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelSpherical::TangentVector_t max_velocity_SO3 =
      JointModelSpherical::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelSpherical::ConfigVector_t lower_position_SO3 =
      JointModelSpherical::ConfigVector_t::Constant(-1.01);
  JointModelSpherical::ConfigVector_t upper_position_SO3 =
      JointModelSpherical::ConfigVector_t::Constant(1.01);

  JointModelRUBZ::TangentVector_t max_effort_SO2 =
      JointModelRUBZ::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelRUBZ::TangentVector_t max_velocity_SO2 =
      JointModelRUBZ::TangentVector_t::Constant(
          std::numeric_limits<double>::max());
  JointModelRUBZ::ConfigVector_t lower_position_SO2 =
      JointModelRUBZ::ConfigVector_t::Constant(-1.01);
  JointModelRUBZ::ConfigVector_t upper_position_SO2 =
      JointModelRUBZ::ConfigVector_t::Constant(1.01);

  JointIndex idJoint = 0;
  idJoint = robot->model().addJoint(
      idJoint, JointModelTranslation(), mat, name + "_xyz", max_effort_tr,
      max_velocity_tr, lower_position_tr, upper_position_tr);
  idJoint = robot->model().addJoint(
      idJoint, JointModelSpherical(), mat, name + "_SO3", max_effort_SO3,
      max_velocity_SO3, lower_position_SO3, upper_position_SO3);
  idJoint = robot->model().addJoint(
      idJoint, JointModelRUBZ(), mat, name + "_SO2", max_effort_SO2,
      max_velocity_SO2, lower_position_SO2, upper_position_SO2);

  robot->createData();
  robot->createGeomData();

  // Build Distance, nearestNeighbor, KDTree
  ProblemPtr_t problem = Problem::create(robot);
  WeighedDistancePtr_t distance = WeighedDistance::create(robot);
  problem->distance(distance);
  ConfigurationShooterPtr_t confShoot = problem->configurationShooter();
  KDTreePtr_t kdTree(std::make_shared<KDTree>(robot, distance, 30));
  hpp::core::nearestNeighbor::Basic basic(distance);
  SteeringMethodPtr_t sm = hpp::core::steeringMethod::Straight::create(problem);

  // Add 4 connectedComponents with 2000 nodes each
  Configuration_t configuration;
  NodePtr_t node;
  NodePtr_t rootNode[4];
  RoadmapPtr_t roadmap = Roadmap::create(distance, robot);
  roadmap->nearestNeighbor(kdTree);
  int opposite = 0;
  for (int i = 0; i < 4; i++) {
    configuration = confShoot->shoot();
    rootNode[i] = roadmap->addNode(configuration);
    for (int j = 1; j < 200; j++) {
      configuration = confShoot->shoot();
      if (opposite == 1) {
	configuration.segment(3,4) *= -1.;
      }
      opposite = 1 -opposite;
      PathPtr_t path = (*sm)(rootNode[i]->configuration(), configuration);
      node = roadmap->addNodeAndEdges(rootNode[i], configuration, path);
      basic.addNode(node);
    }
  }

  // search nearest node
  value_type minDistance1;
  value_type minDistance2;
  NodePtr_t node1;
  NodePtr_t node2;
  for (int j = 0; j < 200; j++) {
    configuration = confShoot->shoot();
    if (opposite == 1) {
      configuration.segment(3,4) *= -1.;
    }
    opposite = 1 - opposite;
    for (int i = 0; i < 4; i++) {
      minDistance1 = std::numeric_limits<value_type>::infinity();
      minDistance2 = std::numeric_limits<value_type>::infinity();
      node1 = basic.search(configuration, rootNode[i]->connectedComponent(),
                           minDistance1);
      node2 = roadmap->nearestNode(
          configuration, rootNode[i]->connectedComponent(), minDistance2);
      BOOST_CHECK(node1 == node2);
      BOOST_CHECK(fabs(minDistance1 - minDistance2) < 1e-15);
      std::cout << displayConfig(node1->configuration()) << std::endl;
      std::cout << minDistance1 << "," << minDistance2 << ","
                << minDistance1 - minDistance2 << std::endl;
    }
  }
}
BOOST_AUTO_TEST_SUITE_END()
