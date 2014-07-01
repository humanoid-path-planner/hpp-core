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
#include <hpp/core/roadmap.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    ProblemSolver::ProblemSolver () :
      robot_ (), robotChanged_ (false), problem_ (),
      initConf_ (), goalConfigurations_ (),
      pathPlannerType_ ("DiffusingPlanner"),
      pathOptimizerType_ ("RandomShortcut"), roadmap_ (), paths_ (),
      pathPlannerFactory_ (), pathOptimizerFactory_ (), constraints_ (),
      collisionObstacles_ (), distanceObstacles_ (), obstacleLoaded_(false),
      errorThreshold_ (1e-4), maxIterations_ (20), NumericalConstraintMap_ ()
    {
      pathOptimizerFactory_ ["RandomShortcut"] = RandomShortcut::create;
      pathPlannerFactory_ ["DiffusingPlanner"] =
	DiffusingPlanner::createWithRoadmap;
    }

    ProblemSolver::~ProblemSolver ()
    {
      if (problem_) delete problem_;
    }

    void ProblemSolver::pathPlannerType (const std::string& type)
    {
      pathPlannerType_ = type;
    }

    void ProblemSolver::pathOptimizerType (const std::string& type)
    {
      pathOptimizerType_ = type;
    }

    void ProblemSolver::robot (const DevicePtr_t& robot)
    {
      robot_ = robot;
      constraints_ = ConstraintSet::create (robot_, "Default constraint set");
      robotChanged_ = true;
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

    void ProblemSolver::resetConstraints ()
    {
      if (robot_)
	constraints_ = ConstraintSet::create (robot_, "Default constraint set");
    }

    void ProblemSolver::addConstraintToConfigProjector(
                          const std::string& constraintName,
                          const DifferentiableFunctionPtr_t& constraint)
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
      configProjector->addConstraint (constraint);
    }

    void ProblemSolver::resetProblem ()
    {
      if (problem_)
	delete problem_;
      problem_ = new Problem (robot_);
      roadmap_ = Roadmap::create (problem_->distance (), problem_->robot());
      problem_->constraints ();
    }

    void ProblemSolver::solve ()
    {
      if (robotChanged_) {
	/// If robot has changed since last call, reset problem and roadmap
	resetProblem ();
      }
      // Set constraints
      problem_->constraints (constraints_);
      // Set obstacles
      problem_->collisionObstacles(collisionObstacles_);
      problem_->distanceObstacles(distanceObstacles_);
      if(obstacleLoaded_) // If collision obstacle added, reset Roadmap
	roadmap_ = Roadmap::create (problem_->distance (), problem_->robot());

      PathPlannerBuilder_t createPlanner =
	pathPlannerFactory_ [pathPlannerType_];
      pathPlanner_ = createPlanner (*problem_, roadmap_);
      PathOptimizerBuilder_t createOptimizer =
	pathOptimizerFactory_ [pathOptimizerType_];
      pathOptimizer_ = createOptimizer (*problem_);
      robotChanged_ = false;
      // Reset init and goal configurations
      problem_->initConfig (initConf_);
      problem_->resetGoalConfigs ();
      obstacleLoaded_=false;
      for (Configurations_t::const_iterator itConfig =
	     goalConfigurations_.begin ();
	   itConfig != goalConfigurations_.end (); itConfig++) {
	problem_->addGoalConfig (*itConfig);
      }
      PathVectorPtr_t path = pathPlanner_->solve ();
      paths_.push_back (path);
      path = pathOptimizer_->optimize (path);
      paths_.push_back (path);
    }

    void ProblemSolver::addObstacle (const CollisionObjectPtr_t& object,
				     bool collision, bool distance)
    {
      
      if (collision){
	collisionObstacles_.push_back (object);
	obstacleLoaded_=true;
	}
      if (distance)
	distanceObstacles_.push_back (object);
      if (problem ())
        problem ()->addObstacle (object, collision, distance);
      obstacleMap_ [object->name ()] = object;
    }

    const CollisionObjectPtr_t& ProblemSolver::obstacle
    (const std::string& name)
    {
      return obstacleMap_ [name];
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
