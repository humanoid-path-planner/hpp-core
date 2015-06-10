//
// Copyright (c) 2014 CNRS
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

#include <hpp/util/debug.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/diffusing-planner.hh>
#include <hpp/core/distance-between-objects.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/continuous-collision-checking/dichotomy.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-optimization/gradient-based.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/visibility-prm-planner.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    // Struct that constructs an empty shared pointer to PathOptimizer.
    struct NoneOptimizer
    {
      static PathOptimizerPtr_t create (const Problem&)
      {
	return PathOptimizerPtr_t ();
      }
    }; // struct NoneOptimizer

    // Struct that constructs an empty shared pointer to PathProjector.
    struct NonePathProjector
    {
      static PathProjectorPtr_t create (const DistancePtr_t&,
					const SteeringMethodPtr_t&, value_type)
      {
	return PathProjectorPtr_t ();
      }
    }; // struct NoneOptimizer

    ProblemSolver::ProblemSolver () :
      constraints_ (), robot_ (), problem_ (),
      initConf_ (), goalConfigurations_ (),
      pathPlannerType_ ("DiffusingPlanner"),
      pathOptimizerTypes_ (),
      pathValidationType_ ("Discretized"), pathValidationTolerance_ (0.05),
      pathProjectorType_ ("None"), pathProjectorTolerance_ (0.2),
      roadmap_ (), paths_ (),
      pathPlannerFactory_ (), pathOptimizerFactory_ (),
      pathValidationFactory_ (),
      collisionObstacles_ (), distanceObstacles_ (),
      errorThreshold_ (1e-4), maxIterations_ (20), numericalConstraintMap_ (),
      passiveDofsMap_ (), distanceBetweenObjects_ ()
    {
      pathOptimizerFactory_ ["RandomShortcut"] = RandomShortcut::create;
      pathOptimizerFactory_ ["GradientBased"] =
	pathOptimization::GradientBased::create;
      pathOptimizerFactory_ ["None"] = NoneOptimizer::create;
      pathPlannerFactory_ ["DiffusingPlanner"] =
	DiffusingPlanner::createWithRoadmap;
      pathPlannerFactory_ ["VisibilityPrmPlanner"] =
	VisibilityPrmPlanner::createWithRoadmap;
      // Store path validation methods in map.
      pathValidationFactory_ ["Discretized"] =
	DiscretizedCollisionChecking::create;
      pathValidationFactory_ ["Progressive"] =
	continuousCollisionChecking::Progressive::create;
      pathValidationFactory_ ["Dichotomy"] =
	continuousCollisionChecking::Dichotomy::create;
      // Store path projector methods in map.
      pathProjectorFactory_ ["None"] =
	NonePathProjector::create;
      pathProjectorFactory_ ["Progressive"] =
	pathProjector::Progressive::create;
      pathProjectorFactory_ ["Dichotomy"] =
	pathProjector::Dichotomy::create;
    }

    ProblemSolver::~ProblemSolver ()
    {
      if (problem_) delete problem_;
    }

    void ProblemSolver::pathPlannerType (const std::string& type)
    {
      if (pathPlannerFactory_.find (type) == pathPlannerFactory_.end ()) {
	throw std::runtime_error (std::string ("No path planner with name ") +
				  type);
      }
      pathPlannerType_ = type;
    }

    void ProblemSolver::configurationShooterType (const std::string& type)
    {
      if (configurationShooterFactory_.find (type) == configurationShooterFactory_.end ()) {
    throw std::runtime_error (std::string ("No configuration shooter with name ") +
                  type);
      }
      configurationShooterType_ = type;
    }

    void ProblemSolver::addPathOptimizer (const std::string& type)
    {
      if (pathOptimizerFactory_.find (type) == pathOptimizerFactory_.end ()) {
	throw std::runtime_error (std::string ("No path optimizer with name ") +
				  type);
      }
      pathOptimizerTypes_.push_back (type);
    }

    void ProblemSolver::clearPathOptimizers ()
    {
      pathOptimizerTypes_.clear ();
      pathOptimizers_.clear ();
    }

    void ProblemSolver::optimizePath (PathVectorPtr_t path)
    {
      createPathOptimizers ();
      for (PathOptimizers_t::const_iterator it = pathOptimizers_.begin ();
	   it != pathOptimizers_.end (); ++it) {
	path = (*it)->optimize (path);
	paths_.push_back (path);
      }
    }

    void ProblemSolver::pathValidationType (const std::string& type,
					    const value_type& tolerance)
    {
      if (pathValidationFactory_.find (type) == pathValidationFactory_.end ()) {
	throw std::runtime_error (std::string ("No path validation method with "
					       "name ") + type);
      }
      pathValidationType_ = type;
      pathValidationTolerance_ = tolerance;
      // If a robot is present, set path validation method
      if (robot_ && problem_) {
	PathValidationPtr_t pathValidation =
	  pathValidationFactory_ [pathValidationType_]
	  (robot_, pathValidationTolerance_);
	problem_->pathValidation (pathValidation);
      }
    }

    void ProblemSolver::pathProjectorType (const std::string& type,
					    const value_type& tolerance)
    {
      if (pathProjectorFactory_.find (type) == pathProjectorFactory_.end ()) {
	throw std::runtime_error (std::string ("No path projector method with "
					       "name ") + type);
      }
      pathProjectorType_ = type;
      pathProjectorTolerance_ = tolerance;
      // If a robot is present, set path projector method
      if (robot_ && problem_) {
	PathProjectorPtr_t pathProjector =
	  pathProjectorFactory_ [pathProjectorType_]
	  (problem_->distance (), problem_->steeringMethod (),
	   pathProjectorTolerance_);
	problem_->pathProjector (pathProjector);
      }
    }

    void ProblemSolver::robot (const DevicePtr_t& robot)
    {
      robot_ = robot;
      constraints_ = ConstraintSet::create (robot_, "Default constraint set");
      resetProblem ();
    }

    const DevicePtr_t& ProblemSolver::robot () const
    {
      return robot_;
    }

    void ProblemSolver::initConfig (const ConfigurationPtr_t& config)
    {
      initConf_ = config;
    }

    const Configurations_t& ProblemSolver::goalConfigs () const
    {
      return goalConfigurations_;
    }

    void ProblemSolver::addGoalConfig (const ConfigurationPtr_t& config)
    {
      goalConfigurations_.push_back (config);
    }

    void ProblemSolver::resetGoalConfigs ()
    {
      goalConfigurations_.clear ();
    }

    void ProblemSolver::addConstraint (const ConstraintPtr_t& constraint)
    {
      if (robot_)
	constraints_->addConstraint (constraint);
      else
	hppDout (error, "Cannot add constraint while robot is not set");
    }

    void ProblemSolver::addLockedJoint (const LockedJointPtr_t& lj)
    {
      if (!robot_) {
	hppDout (error, "Cannot add constraint while robot is not set");
      }
      ConfigProjectorPtr_t  configProjector = constraints_->configProjector ();
      if (!configProjector) {
	configProjector = ConfigProjector::create
	  (robot_, "ConfigProjector", errorThreshold_, maxIterations_);
	constraints_->addConstraint (configProjector);
      }
      configProjector->add (lj);
    }

    void ProblemSolver::resetConstraints ()
    {
      if (robot_)
	constraints_ = ConstraintSet::create (robot_, "Default constraint set");
    }

    void ProblemSolver::addFunctionToConfigProjector
    (const std::string& constraintName, const std::string& functionName)
    {
      if (!robot_) {
	hppDout (error, "Cannot add constraint while robot is not set");
      }
      ConfigProjectorPtr_t  configProjector = constraints_->configProjector ();
      if (!configProjector) {
	configProjector = ConfigProjector::create
	  (robot_, constraintName, errorThreshold_, maxIterations_);
	constraints_->addConstraint (configProjector);
      }
      configProjector->add (NumericalConstraint::create
          (numericalConstraintMap_ [functionName],
           comparisonTypeMap_ [functionName]));
    }

    void ProblemSolver::resetProblem ()
    {
      if (problem_)
	delete problem_;
      initializeProblem (new Problem (robot_));
    }

    void ProblemSolver::initializeProblem (ProblemPtr_t problem)
    {
      problem_ = problem;
      resetRoadmap ();
      // Set constraints
      problem_->constraints (constraints_);
      // Set path validation method
      PathValidationPtr_t pathValidation =
	pathValidationFactory_ [pathValidationType_] (robot_,
						      pathValidationTolerance_);
      problem_->pathValidation (pathValidation);
      // Set obstacles
      problem_->collisionObstacles(collisionObstacles_);
      // Distance to obstacles
      distanceBetweenObjects_ = DistanceBetweenObjectsPtr_t
	(new DistanceBetweenObjects (robot_));
      distanceBetweenObjects_->obstacles(distanceObstacles_);
    }

    void ProblemSolver::resetRoadmap ()
    {
      if (!problem_)
        throw std::runtime_error ("The problem is not defined.");
      roadmap_ = Roadmap::create (problem_->distance (), problem_->robot());
    }

    void ProblemSolver::createPathOptimizers ()
    {
      if (pathOptimizers_.size () == 0) {
	for (PathOptimizerTypes_t::const_iterator it =
	       pathOptimizerTypes_.begin (); it != pathOptimizerTypes_.end ();
	     ++it) {
	  PathOptimizerBuilder_t createOptimizer = pathOptimizerFactory_ [*it];
	  pathOptimizers_.push_back (createOptimizer (*problem_));
	}
      }
    }

    bool ProblemSolver::prepareSolveStepByStep ()
    {
      PathPlannerBuilder_t createPlanner =
	pathPlannerFactory_ [pathPlannerType_];
      pathPlanner_ = createPlanner (*problem_, roadmap_);
      /// create Path projector
      PathProjectorBuilder_t createProjector =
        pathProjectorFactory_ [pathProjectorType_];
      // Create a default steering method until we add a steering method
      // factory.
      SteeringMethodPtr_t sm (SteeringMethodStraight::create (robot ()));
      PathProjectorPtr_t pathProjector_ =
        createProjector (problem_->distance (), sm, pathProjectorTolerance_);
      problem_->pathProjector (pathProjector_);
      // Reset init and goal configurations
      problem_->initConfig (initConf_);
      problem_->resetGoalConfigs ();
      for (Configurations_t::const_iterator itConfig =
	     goalConfigurations_.begin ();
	   itConfig != goalConfigurations_.end (); ++itConfig) {
	problem_->addGoalConfig (*itConfig);
      }

      pathPlanner_->startSolve ();
      pathPlanner_->tryDirectPath ();
      return roadmap_->pathExists ();
    }

    bool ProblemSolver::executeOneStep ()
    {
      pathPlanner_->oneStep ();
      return roadmap_->pathExists ();
    }

    void ProblemSolver::finishSolveStepByStep ()
    {
      if (!roadmap_->pathExists ())
        throw std::logic_error ("No path exists.");
      PathVectorPtr_t planned =  pathPlanner_->computePath ();
      paths_.push_back (pathPlanner_->finishSolve (planned));
    }

    void ProblemSolver::solve ()
    {
      PathPlannerBuilder_t createPlanner =
	pathPlannerFactory_ [pathPlannerType_];
      pathPlanner_ = createPlanner (*problem_, roadmap_);
      /// create Path projector
      PathProjectorBuilder_t createProjector =
        pathProjectorFactory_ [pathProjectorType_];
      // Create a default steering method until we add a steering method
      // factory.
      SteeringMethodPtr_t sm (SteeringMethodStraight::create (robot ()));
      PathProjectorPtr_t pathProjector_ =
        createProjector (problem_->distance (), sm, pathProjectorTolerance_);
      problem_->pathProjector (pathProjector_);
      /// create Path optimizer
      // Reset init and goal configurations
      problem_->initConfig (initConf_);
      problem_->resetGoalConfigs ();
      for (Configurations_t::const_iterator itConfig =
	     goalConfigurations_.begin ();
	   itConfig != goalConfigurations_.end (); ++itConfig) {
	problem_->addGoalConfig (*itConfig);
      }
      PathVectorPtr_t path = pathPlanner_->solve ();
      paths_.push_back (path);
      optimizePath (path);
    }

    void ProblemSolver::addObstacle (const CollisionObjectPtr_t& object,
				     bool collision, bool distance)
    {

      if (collision){
	collisionObstacles_.push_back (object);
        resetRoadmap ();
      }
      if (distance)
	distanceObstacles_.push_back (object);
      if (problem ())
        problem ()->addObstacle (object);
      if (distanceBetweenObjects_) {
	distanceBetweenObjects_->addObstacle (object);
      }
      obstacleMap_ [object->name ()] = object;
    }

    void ProblemSolver::removeObstacleFromJoint
    (const std::string& obstacleName, const std::string& jointName)
    {
      if (!robot_) {
	throw std::runtime_error ("No robot defined.");
      }
      JointPtr_t joint = robot_->getJointByName (jointName);
      const CollisionObjectPtr_t& object = obstacle (obstacleName);
      problem ()->removeObstacleFromJoint (joint, object);
    }

    const CollisionObjectPtr_t& ProblemSolver::obstacle
    (const std::string& name)
    {
      return obstacleMap_ [name];
    }

    std::list <std::string> ProblemSolver::obstacleNames
    (bool collision, bool distance) const
    {
      std::list <std::string> res;
      if (collision) {
	for (ObjectVector_t::const_iterator it = collisionObstacles_.begin ();
	     it != collisionObstacles_.end (); ++it) {
	  res.push_back ((*it)->name ());
	}
      }
      if (distance) {
	for (ObjectVector_t::const_iterator it = distanceObstacles_.begin ();
	     it != distanceObstacles_.end (); ++it) {
	  res.push_back ((*it)->name ());
	}
      }
      return res;
    }

    const ObjectVector_t& ProblemSolver::collisionObstacles () const
    {
      return collisionObstacles_;
    }


    const ObjectVector_t& ProblemSolver::distanceObstacles () const
    {
      return distanceObstacles_;
    }

  } //   namespace core
} // namespace hpp
