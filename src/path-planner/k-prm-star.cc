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

#include <hpp/core/path-planner/k-prm-star.hh>

#include <cmath>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-projector.hh>
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
        numberNodes_ = problem().getParameter ("kPRM*/numberOfNodes").intValue();
        if (numberNodes_ == 0) {
          std::ostringstream oss;
          oss << "kPrmStar: Number nodes should be positive, got "
              << numberNodes_;
          throw std::runtime_error (oss.str ().c_str ());
        }
        numberNeighbors_ = (size_type) floor
          ((kPRM * log ((value_type) numberNodes_)) + .5);
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
            break;
          case LINK_NODES:
            linkNodes ();
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
	Configuration_t qrand;
	// Report of configuration validation: unused here
	ValidationReportPtr_t validationReport;
	// Configuration validation methods associated to the problem
	ConfigValidationsPtr_t configValidations
          (problem ().configValidations ());
	// Get the constraints the robot is subject to
	ConstraintSetPtr_t constraints (problem ().constraints ());
        // Get the problem shooter
        ConfigurationShooterPtr_t shooter = problem().configurationShooter();
	// Get roadmap
	RoadmapPtr_t r (roadmap ());
        if (r->nodes ().size () < numberNodes_) {
          size_type nbTry = 0;
          bool valid (false);
          // After 10000 trials throw if no valid configuration has been found.
          do {
            shooter->shoot (qrand);
            valid = (!constraints || constraints->apply (qrand));
            if (valid)
              valid = configValidations->validate (qrand, validationReport);
	    nbTry++;
          } while (!valid && nbTry < 10000);
          if (!valid) {
            throw std::runtime_error
              ("Failed to generate free configuration after 10000 trials.");
          }
          r->addNode (qrand);
        } else {
          state_ = LINK_NODES;
          linkingNodeIt_ = r->nodes ().begin ();
          neighbors_ =roadmap ()->nearestNodes
            (*(*linkingNodeIt_)->configuration (), numberNeighbors_);
          itNeighbor_ = neighbors_.begin ();
        }
      }

      void kPrmStar::linkNodes ()
      {
	// Get roadmap
	RoadmapPtr_t r (roadmap ());
        if (linkingNodeIt_ != r->nodes ().end ()) {
          if (connectNodeToClosestNeighbors (*linkingNodeIt_)) {
            ++linkingNodeIt_;
	    if (linkingNodeIt_ != r->nodes ().end ()) {
	      neighbors_ = roadmap ()->nearestNodes
		((*linkingNodeIt_)->configuration (), numberNeighbors_);
	      // Connect current node with closest neighbors
	      itNeighbor_ = neighbors_.begin ();
	    }
          } else {
            ++itNeighbor_;
          }
        } else {
          state_ = CONNECT_INIT_GOAL;
        }
      }

      bool kPrmStar::connectNodeToClosestNeighbors (const NodePtr_t& node)
      {
	// Retrieve the path validation algorithm associated to the problem
	PathValidationPtr_t pathValidation (problem ().pathValidation ());
	// Retrieve the steering method
	SteeringMethodPtr_t sm (problem ().steeringMethod ());
	// Retrieve the constraints the robot is subject to
	ConstraintSetPtr_t constraints (problem ().constraints ());
        // Retrieve path projector
        PathProjectorPtr_t pathProjector (problem ().pathProjector ());

        if (itNeighbor_ != neighbors_.end ()) {
          // Connect only nodes that are not already connected
          if (!(*itNeighbor_)->isOutNeighbor (node) && (node != *itNeighbor_)) {
            PathPtr_t p ((*sm) (*node->configuration (),
                                *(*itNeighbor_)->configuration ()));
            PathValidationReportPtr_t report;
            PathPtr_t validPart, projected;
            if (p) {
              bool success;
              if (pathProjector) {
                success = pathProjector->apply (p, projected);
              } else {
                projected = p;
                success = true;
              }
              if (success) {
                if (pathValidation->validate (projected, false, validPart,
                                              report)) {
                  roadmap ()->addEdges (node, *itNeighbor_, projected);
                }
              }
            }
          }
          return false;
        } else {
          // itNeighbor_ reached the end
          return true;
        }
      }

      void kPrmStar::connectInitAndGoal ()
      {
        NodePtr_t initNode (roadmap ()->initNode ());
        if (initNode->outEdges ().empty ()) {
          neighbors_ = roadmap ()->nearestNodes
            (initNode->configuration (), numberNeighbors_);
          // Connect current node with closest neighbors
          for (itNeighbor_ = neighbors_.begin ();
               itNeighbor_ != neighbors_.end (); ++itNeighbor_) {
            connectNodeToClosestNeighbors (initNode);
          }
        }
        for (NodeVector_t::const_iterator itn
               (roadmap ()->goalNodes ().begin ());
             itn != roadmap ()->goalNodes ().end (); ++itn) {
          neighbors_ = roadmap ()->nearestNodes
            ((*itn)->configuration (), numberNeighbors_);
          if ((*itn)->inEdges ().empty ()) {
            for (itNeighbor_ = neighbors_.begin ();
                 itNeighbor_ != neighbors_.end (); ++itNeighbor_) {
              connectNodeToClosestNeighbors (*itn);
            }
          }
        }
      }

      kPrmStar::STATE kPrmStar::getComputationState () const
      {
	return state_;
      }

      kPrmStar::kPrmStar (const Problem& problem) :
        Parent_t (problem),
        state_ (BUILD_ROADMAP)
      {}

      kPrmStar::kPrmStar (const Problem& problem, const RoadmapPtr_t& roadmap) :
        Parent_t (problem, roadmap),
        state_ (BUILD_ROADMAP)
      {}

      void kPrmStar::init (const kPrmStarWkPtr_t& weak)
      {
        Parent_t::init (weak);
        weak_ = weak;
      }

      // ----------- Declare parameters ------------------------------------- //

      HPP_START_PARAMETER_DECLARATION(kPrmStar)
      Problem::declareParameter(ParameterDescription (Parameter::INT,
            "kPRM*/numberOfNodes",
            "The desired number of nodes in the roadmap.",
            Parameter((size_type)100)));
      HPP_END_PARAMETER_DECLARATION(kPrmStar)
    } // namespace pathPlanner
  } // namespace core
} // namespace hpp
