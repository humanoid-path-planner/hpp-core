//
// Copyright (c) 2018 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <cmath>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

namespace hpp {
  namespace core {
    namespace pathPlanner {

      const double kPrmStar::kPRM = 2 * exp (1);

      kPrmStarPtr_t kPrmStar::create (const Problem& problem)
      {
        kPrmStarPtr_t shPtr (new kPrmStar (problem));
        shPtr->init (shPtr);
        return shPtr;
      }

      kPrmStarPtr_t kPrmStar::createWithRoadmap (const Problem& problem,
                                                 const RoadmapPtr_t& roadmap)
      {
        kPrmStarPtr_t shPtr (new kPrmStar (problem, roadmap));
        shPtr->init (shPtr);
        return shPtr;
      }

      void kPrmStar::startSolve ()
      {
        Parent_t::startSolve ();
        numberNodes_ = problem().getParameter <size_type>
          ("kPRMstar/numberOfNodes", 100);
        if (numberNodes_ == 0) {
          std::ostringstream oss;
          oss << "kPrmStar: Number nodes should be positive, got "
              << numberNodes_;
          throw std::runtime_error (oss.str ().c_str ());
        }
        if (roadmap ()->nodes ().size () >= numberNodes_) {
          state_ = CONNECT_INIT_GOAL;
        } else {
          state_ = BUILD_ROADMAP;
        }
      }

      void kPrmStar::oneStep ()
      {
        std::ostringstream oss;
        // Shoot valid random configurations
        switch (state_)
          {
          case BUILD_ROADMAP:
            generateRandomConfig ();
            linkNodes ();
            state_ = FAILURE;
            break;
          case CONNECT_INIT_GOAL:
            connectInitAndGoal ();
            state_ = FAILURE;
            break;
          case FAILURE:
            oss << "kPRM* failed to solve problem with " << numberNodes_
                << " nodes.";
            throw std::runtime_error (oss.str ().c_str ());
          }
      }

      void kPrmStar::generateRandomConfig ()
      {
	// shoot a valid random configuration
	ConfigurationPtr_t qrand;
	// Report of configuration validation: unused here
	ValidationReportPtr_t validationReport;
	// Configuration validation methods associated to the problem
	ConfigValidationsPtr_t configValidations
          (problem ().configValidations ());
	// Get roadmap
	RoadmapPtr_t r (roadmap ());
        while (r->nodes ().size () < numberNodes_) {
          size_type nbTry = 0;
          bool valid;
          // After 10000 trials throw if no valid configuration has been found.
          do {
            qrand = shooter_->shoot ();
            valid = configValidations->validate (*qrand, validationReport);
          } while (!valid && nbTry < 10000);
          if (!valid) {
            throw std::runtime_error
              ("Failed to generate free configuration after 10000 trials.");
          }
          r->addNode (qrand);
        }
      }

      void kPrmStar::linkNodes ()
      {
	// Get roadmap
	RoadmapPtr_t r (roadmap ());
        for (Nodes_t::const_iterator linkingNodeIt = r->nodes ().begin ();
             linkingNodeIt != r->nodes ().end (); ++linkingNodeIt) {
          // Connect current node with closest neighbors
          connectNodeToClosestNeighbors (*linkingNodeIt);
        }
      }

      void kPrmStar::connectNodeToClosestNeighbors (const NodePtr_t& node)
      {
	// Retrieve the path validation algorithm associated to the problem
	PathValidationPtr_t pathValidation (problem ().pathValidation ());
	// Retrieve the steering method
	SteeringMethodPtr_t sm (problem ().steeringMethod ());
	// Retrieve the constraints the robot is subject to
	ConstraintSetPtr_t constraints (problem ().constraints ());

        size_type numberNeighbors
          ((size_type) floor ((kPRM * log ((value_type) numberNodes_)) + .5));
        Nodes_t neighbors
          (roadmap ()->nearestNodes (node->configuration (), numberNeighbors));
        for (Nodes_t::iterator it = neighbors.begin ();
             it != neighbors.end (); ++it) {
          // Connect only nodes that are not already connected
          if (!(*it)->isOutNeighbor (node) && (node != *it)) {
            PathPtr_t p ((*sm) (*node->configuration (),
                                *(*it)->configuration ()));
            PathValidationReportPtr_t report;
            PathPtr_t validPart;
            if (p && pathValidation->validate (p, false, validPart, report)) {
              roadmap ()->addEdges (node, *it, p);
            }
          }
        }
      }

      void kPrmStar::connectInitAndGoal ()
      {
        NodePtr_t node (roadmap ()->initNode ());
        if (node->outEdges ().empty ()) {
          connectNodeToClosestNeighbors (node);
        }
        for (NodeVector_t::const_iterator itn
               (roadmap ()->goalNodes ().begin ());
             itn != roadmap ()->goalNodes ().end (); ++itn) {
          if ((*itn)->inEdges ().empty ()) {
            connectNodeToClosestNeighbors (*itn);
          }
        }
      }

      kPrmStar::kPrmStar (const Problem& problem) :
        Parent_t (problem),
        state_ (BUILD_ROADMAP),
        shooter_ (BasicConfigurationShooter::create (problem.robot ()))
      {
      }

      kPrmStar::kPrmStar (const Problem& problem, const RoadmapPtr_t& roadmap) :
        Parent_t (problem, roadmap),
        state_ (BUILD_ROADMAP),
        shooter_ (BasicConfigurationShooter::create (problem.robot ()))
      {
      }

      void kPrmStar::init (const kPrmStarWkPtr_t& weak)
      {
        Parent_t::init (weak);
        weak_ = weak;
      }
    } // namespace pathPlanner
  } // namespace core
} // namespace hpp
