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

#include <hpp/core/problem-solver.hh>

#include <boost/bind.hpp>

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/collision-object.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/continuous-collision-checking/dichotomy.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/diffusing-planner.hh>
#include <hpp/core/distance-between-objects.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/numerical-constraint.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-optimization/gradient-based.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-optimization/config-optimization.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem-target/task-target.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/visibility-prm-planner.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    using pinocchio::Model;
    using pinocchio::GeomModel;
    using pinocchio::GeomModelPtr_t;
    using pinocchio::GeomData;
    using pinocchio::GeomDataPtr_t;
    using pinocchio::CollisionObject;

    namespace {
      Model initObstacleModel () {
        Model model;
        model.addFrame(se3::Frame("obstacle_frame", 0, 0, Transform3f::Identity(), se3::BODY));
        return model;
      }

      const Model obsModel = initObstacleModel();
    }

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
    }; // struct NonePathProjector

    ProblemSolverPtr_t ProblemSolver::latest_ = 0x0;
    ProblemSolverPtr_t ProblemSolver::create ()
    {
      latest_ = new ProblemSolver ();
      return latest_;
    }

    ProblemSolverPtr_t ProblemSolver::latest ()
    {
      return latest_;
    }

    ProblemSolver::ProblemSolver () :
      constraints_ (), robot_ (), problem_ (NULL), pathPlanner_ (),
      roadmap_ (), paths_ (),
      pathProjectorType_ ("None"), pathProjectorTolerance_ (0.2),
      pathPlannerType_ ("DiffusingPlanner"),
      initConf_ (), goalConfigurations_ (),
      configurationShooterType_ ("BasicConfigurationShooter"),
      distanceType_("WeighedDistance"),
      steeringMethodType_ ("SteeringMethodStraight"),
      pathOptimizerTypes_ (), pathOptimizers_ (),
      pathValidationType_ ("Discretized"), pathValidationTolerance_ (0.05),
      collisionObstacles_ (), distanceObstacles_ (),
      obstacleModel_ (new GeomModel()), obstacleData_ (new GeomData(*obstacleModel_)),
      errorThreshold_ (1e-4), maxIterations_ (20),
      passiveDofsMap_ (), comcMap_ (),
      distanceBetweenObjects_ ()
    {
      add <PathPlannerBuilder_t> ("DiffusingPlanner",     DiffusingPlanner::createWithRoadmap);
      add <PathPlannerBuilder_t> ("VisibilityPrmPlanner", VisibilityPrmPlanner::createWithRoadmap);
      add <PathPlannerBuilder_t> ("BiRRTPlanner", BiRRTPlanner::createWithRoadmap);

      add <ConfigurationShooterBuilder_t> ("BasicConfigurationShooter", BasicConfigurationShooter::create);
      add <DistanceBuilder_t> ("WeighedDistance",
			       WeighedDistance::createFromProblem);
      add <SteeringMethodBuilder_t> ("SteeringMethodStraight", boost::bind(
            static_cast<SteeringMethodStraightPtr_t (*)(const ProblemPtr_t&)>
              (&SteeringMethodStraight::create), _1
            ));
      add <SteeringMethodBuilder_t> ("ReedsShepp", steeringMethod::ReedsShepp::createWithGuess);

      // Store path optimization methods in map.
      add <PathOptimizerBuilder_t> ("RandomShortcut",     RandomShortcut::create);
      add <PathOptimizerBuilder_t> ("GradientBased",      pathOptimization::GradientBased::create);
      add <PathOptimizerBuilder_t> ("PartialShortcut",    pathOptimization::PartialShortcut::create);
      add <PathOptimizerBuilder_t> ("ConfigOptimization", pathOptimization::ConfigOptimization::create);
      add <PathOptimizerBuilder_t> ("None",               NoneOptimizer::create); // TODO: Delete me

      // Store path validation methods in map.
      add <PathValidationBuilder_t> ("Discretized", DiscretizedCollisionChecking::create);
      add <PathValidationBuilder_t> ("Progressive", continuousCollisionChecking::Progressive::create);
      add <PathValidationBuilder_t> ("Dichotomy",   continuousCollisionChecking::Dichotomy::create);

      // Store path projector methods in map.
      add <PathProjectorBuilder_t> ("None",        NonePathProjector::create);
      add <PathProjectorBuilder_t> ("Progressive", pathProjector::Progressive::create);
      add <PathProjectorBuilder_t> ("Dichotomy",   pathProjector::Dichotomy::create);
      add <PathProjectorBuilder_t> ("Global",      pathProjector::Global::create);
    }

    ProblemSolver::~ProblemSolver ()
    {
      if (problem_) delete problem_;
    }

    void ProblemSolver::distanceType (const std::string& type)
    {
      if (!has <DistanceBuilder_t> (type)) {
    throw std::runtime_error (std::string ("No distance method with name ") +
                  type);
      }
      distanceType_ = type;
    }

    void ProblemSolver::steeringMethodType (const std::string& type)
    {
      if (!has <SteeringMethodBuilder_t> (type)) {
	throw std::runtime_error (std::string ("No steering method with name ") +
				  type);
      }
      steeringMethodType_ = type;
    }

    void ProblemSolver::pathPlannerType (const std::string& type)
    {
      if (!has <PathPlannerBuilder_t> (type)) {
	throw std::runtime_error (std::string ("No path planner with name ") +
				  type);
      }
      pathPlannerType_ = type;
    }

    void ProblemSolver::configurationShooterType (const std::string& type)
    {
      if (!has <ConfigurationShooterBuilder_t> (type)) {
    throw std::runtime_error (std::string ("No configuration shooter with name ") +
                  type);
      }
      configurationShooterType_ = type;
    }

    void ProblemSolver::addPathOptimizer (const std::string& type)
    {
      if (!has <PathOptimizerBuilder_t> (type)) {
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
      if (!has <PathValidationBuilder_t> (type)) {
	throw std::runtime_error (std::string ("No path validation method with "
					       "name ") + type);
      }
      pathValidationType_ = type;
      pathValidationTolerance_ = tolerance;
      // If a robot is present, set path validation method
      if (robot_ && problem_) {
	PathValidationPtr_t pathValidation =
	  get <PathValidationBuilder_t> (pathValidationType_)
	  (robot_, pathValidationTolerance_);
	problem_->pathValidation (pathValidation);
      }
    }

    void ProblemSolver::pathProjectorType (const std::string& type,
					    const value_type& tolerance)
    {
      if (!has <PathProjectorBuilder_t> (type)) {
	throw std::runtime_error (std::string ("No path projector method with "
					       "name ") + type);
      }
      pathProjectorType_ = type;
      pathProjectorTolerance_ = tolerance;
      // If a robot is present, set path projector method
      if (robot_ && problem_) {
	initPathProjector ();
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

    void ProblemSolver::addGoalConstraint (const ConstraintPtr_t& constraint)
    {
      if (!goalConstraints_) {
        if (!robot_) throw std::logic_error ("You must provide a robot first");
        goalConstraints_ = ConstraintSet::create (robot_, "Goal constraint set");
      }
      goalConstraints_->addConstraint (constraint);
    }

    void ProblemSolver::addGoalConstraint (const LockedJointPtr_t& lj)
    {
      if (!goalConstraints_) {
        if (!robot_) throw std::logic_error ("You must provide a robot first");
        goalConstraints_ = ConstraintSet::create (robot_, "Goal constraint set");
      }
      ConfigProjectorPtr_t  configProjector = goalConstraints_->configProjector ();
      if (!configProjector) {
	configProjector = ConfigProjector::create
	  (robot_, "Goal ConfigProjector", errorThreshold_, maxIterations_);
	goalConstraints_->addConstraint (configProjector);
      }
      configProjector->add (lj);
    }

    void ProblemSolver::addGoalConstraint (const std::string& constraintName,
        const std::string& functionName, const std::size_t priority)
    {
      if (!goalConstraints_) {
        if (!robot_) throw std::logic_error ("You must provide a robot first");
        goalConstraints_ = ConstraintSet::create (robot_, "Goal constraint set");
      }
      ConfigProjectorPtr_t  configProjector = goalConstraints_->configProjector ();
      if (!configProjector) {
	configProjector = ConfigProjector::create
	  (robot_, constraintName, errorThreshold_, maxIterations_);
	goalConstraints_->addConstraint (configProjector);
      }
      configProjector->add (numericalConstraint (functionName),
			    SizeIntervals_t (0), priority);
    }

    void ProblemSolver::resetGoalConstraint ()
    {
      goalConstraints_.reset ();
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
    (const std::string& constraintName, const std::string& functionName,
     const std::size_t priority)
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
      if (!has <NumericalConstraintPtr_t> (functionName)) {
        std::stringstream ss; ss << "Function " << functionName << " does not exists";
        throw std::invalid_argument (ss.str());
      }
      configProjector->add (get<NumericalConstraintPtr_t> (functionName),
			    SizeIntervals_t (0), priority);
    }

    void ProblemSolver::comparisonType (const std::string& name,
        const ComparisonType::VectorOfTypes types)
    {
      if (!has <NumericalConstraintPtr_t> (name))
        throw std::logic_error (std::string ("Numerical constraint ") +
            name + std::string (" not defined."));
      ComparisonTypesPtr_t eqtypes = ComparisonTypes::create (types);
      get<NumericalConstraintPtr_t> (name)->comparisonType (eqtypes);
    }

    void ProblemSolver::comparisonType (const std::string& name,
        const ComparisonTypePtr_t eq)
    {
      if (!has <NumericalConstraintPtr_t> (name))
        throw std::logic_error (std::string ("Numerical constraint ") +
            name + std::string (" not defined."));
      get<NumericalConstraintPtr_t> (name)->comparisonType (eq);
    }

    ComparisonTypePtr_t ProblemSolver::comparisonType (const std::string& name) const
    {
      if (!has <NumericalConstraintPtr_t> (name))
        throw std::logic_error (std::string ("Numerical constraint ") +
            name + std::string (" not defined."));
      return get<NumericalConstraintPtr_t> (name)->comparisonType ();
    }

    void ProblemSolver::computeValueAndJacobian
    (const Configuration_t& configuration, vector_t& value, matrix_t& jacobian)
      const
    {
      if (!robot ()) throw std::runtime_error ("No robot loaded");
      ConfigProjectorPtr_t configProjector
	(constraints ()->configProjector ());
      if (!configProjector) {
	throw std::runtime_error ("No constraints have assigned.");
      }
      // resize value and Jacobian
      NumericalConstraints_t constraints
	(configProjector->numericalConstraints ());
      size_type rows = 0;
      for (NumericalConstraints_t::const_iterator it = constraints.begin ();
	   it != constraints.end (); ++it) {
	rows += (*it)->function ().outputSize ();
      }
      jacobian.resize (rows, configProjector->numberNonLockedDof ());
      value.resize (rows);
      configProjector->computeValueAndJacobian (configuration, value, jacobian);
    }

    void ProblemSolver::maxIterations (size_type iterations)
    {
      maxIterations_ = iterations;
      if (constraints_ && constraints_->configProjector ()) {
        constraints_->configProjector ()->maxIterations (iterations);
      }
    }

    void ProblemSolver::errorThreshold (const value_type& threshold)
    {
      errorThreshold_ = threshold;
      if (constraints_ && constraints_->configProjector ()) {
        constraints_->configProjector ()->errorThreshold (threshold);
      }
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
	get <PathValidationBuilder_t> (pathValidationType_)
        (robot_, pathValidationTolerance_);
      problem_->pathValidation (pathValidation);
      // Set obstacles
      problem_->collisionObstacles(collisionObstacles_);
      // Distance to obstacles
      distanceBetweenObjects_ = DistanceBetweenObjectsPtr_t
	(new DistanceBetweenObjects (robot_));
      distanceBetweenObjects_->obstacles(distanceObstacles_);
    }

    void ProblemSolver::problem (ProblemPtr_t problem)
    {
      if (problem_)
        delete problem_;
      problem_ = problem;
    }


    void ProblemSolver::resetRoadmap ()
    {
      if (!problem_)
        throw std::runtime_error ("The problem is not defined.");
      roadmap_ = Roadmap::create (problem_->distance (), problem_->robot());
    }

    void ProblemSolver::createPathOptimizers ()
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");
      if (pathOptimizers_.size () == 0) {
	for (PathOptimizerTypes_t::const_iterator it =
	       pathOptimizerTypes_.begin (); it != pathOptimizerTypes_.end ();
	     ++it) {
	  PathOptimizerBuilder_t createOptimizer =
            get<PathOptimizerBuilder_t> (*it);
	  pathOptimizers_.push_back (createOptimizer (*problem_));
	}
      }
    }

    void ProblemSolver::initDistance ()
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");
      DistancePtr_t dist (
          get <DistanceBuilder_t> (distanceType_) (problem_)
          );
      problem_->distance (dist);
    }


    void ProblemSolver::initSteeringMethod ()
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");
      SteeringMethodPtr_t sm (
          get <SteeringMethodBuilder_t> (steeringMethodType_) (problem_)
          );
      problem_->steeringMethod (sm);
    }

    void ProblemSolver::initPathProjector ()
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");
      PathProjectorBuilder_t createProjector =
        get <PathProjectorBuilder_t> (pathProjectorType_);
      // Create a default steering method until we add a steering method
      // factory.
      // TODO: the steering method of a path projector should not have
      //       the problem constraints.
      PathProjectorPtr_t pathProjector_ =
        createProjector (problem_->distance (), 
            SteeringMethodStraight::create (problem_),
            pathProjectorTolerance_);
      problem_->pathProjector (pathProjector_);
    }

    void ProblemSolver::initProblem ()
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");


      // Set shooter
      problem_->configurationShooter
        (get <ConfigurationShooterBuilder_t> (configurationShooterType_) (robot_));
      // set distance
      initDistance();
      // build roadmap with new distance
      resetRoadmap();
      // Set steeringMethod
      initSteeringMethod ();
      PathPlannerBuilder_t createPlanner =
        get <PathPlannerBuilder_t> (pathPlannerType_);
      pathPlanner_ = createPlanner (*problem_, roadmap_);
      roadmap_ = pathPlanner_->roadmap();
      /// create Path projector
      initPathProjector ();
      /// create Path optimizer
      // Reset init and goal configurations
      problem_->initConfig (initConf_);
      ProblemTargetPtr_t target;
      if (goalConstraints_) {
        problemTarget::TaskTargetPtr_t t = problemTarget::TaskTarget::create (pathPlanner_);
        t->constraints (goalConstraints_);
        target = t;
      } else
        target = problemTarget::GoalConfigurations::create (pathPlanner_);
      problem_->target (target);
      for (Configurations_t::const_iterator itConfig =
	     goalConfigurations_.begin ();
	   itConfig != goalConfigurations_.end (); ++itConfig) {
	target->addGoalConfig (*itConfig);
      }
    }

    bool ProblemSolver::prepareSolveStepByStep ()
    {
      initProblem ();

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
      initProblem ();

      PathVectorPtr_t path = pathPlanner_->solve ();
      paths_.push_back (path);
      optimizePath (path);
    }

    bool ProblemSolver::directPath
    (ConfigurationIn_t start, ConfigurationIn_t end, bool validate,
     std::size_t& pathId, std::string& report)
    {
      if (!problem_) throw std::runtime_error ("The problem is not defined.");

      // Create steering method using factory
      SteeringMethodPtr_t sm (get <SteeringMethodBuilder_t> 
                             (steeringMethodType_) (problem_));
      problem_->steeringMethod (sm);
      PathPtr_t dp = (*sm) (start, end);
      if (!dp) {
	report = "Steering method failed to build a path.";
	pathId = -1;
	return false;
      }
      PathPtr_t validSection;
      PathValidationReportPtr_t r;
      bool PathValid = true;
      if (validate) {
	PathValid = problem()->pathValidation ()->validate
	  (dp, false, validSection, r);
      }
      if (!PathValid) {
	hppDout(info, "Path only partly valid!");
	hppDout (info, *r);
	dp = validSection;
	std::ostringstream oss;
	oss << *r; report = oss.str ();
      } else {
	report = "";
      }
      // Add Path in problem
      PathVectorPtr_t path (core::PathVector::create (dp->outputSize (),
		 	   dp->outputDerivativeSize ()));
      path->appendPath (dp);
      pathId = addPath (path);
      return PathValid;
    }

    void ProblemSolver::addConfigToRoadmap (const ConfigurationPtr_t& config) 
    {
      roadmap_->addNode(config);
    }  

    void ProblemSolver::addEdgeToRoadmap (const ConfigurationPtr_t& config1,
					  const ConfigurationPtr_t& config2,
					  const PathPtr_t& path)
    {
      NodePtr_t node1, node2;
      value_type accuracy = 10e-6;
      value_type distance1, distance2;
      node1 = roadmap_->nearestNode(config1, distance1);
      node2 = roadmap_->nearestNode(config2, distance2);
      if (distance1 >= accuracy) {
	throw std::runtime_error ("No node of the roadmap contains config1");
      }
      if (distance2 >= accuracy) {
	throw std::runtime_error ("No node of the roadmap contains config2");
      }
      roadmap_->addEdge(node1, node2, path);
    }

    void ProblemSolver::interrupt ()
    {
      if (pathPlanner ()) pathPlanner ()->interrupt ();
      for (PathOptimizers_t::iterator it = pathOptimizers_.begin ();
	   it != pathOptimizers_.end (); ++it) {
	(*it)->interrupt ();
      }
    }

    void ProblemSolver::addObstacle (const CollisionObjectPtr_t& object,
				     bool collision, bool distance)
    {
      // FIXME propagate object->mesh_path ?
      addObstacle(object->name(), *object->fcl(), collision, distance);
    }

    void ProblemSolver::addObstacle (const std::string& name,
                                     /*const*/ FclCollisionObject &inObject,
				     bool collision, bool distance)
    {
      if (obstacleModel_->existGeometryName(name)) {
        HPP_THROW(std::runtime_error, "object with name " << name
            << " already added! Choose another name (prefix).");
      }

      se3::GeomIndex id = obstacleModel_->addGeometryObject(se3::GeometryObject(
            name, obsModel.getFrameId("obstacle_frame", se3::BODY), 0,
            inObject.collisionGeometry(),
            se3::toPinocchioSE3(inObject.getTransform()),
            ""),
          obsModel);
      // Update obstacleData_
      // FIXME This should be done in Pinocchio
      {
        se3::GeometryModel& model = *obstacleModel_;
        se3::GeometryData& data = *obstacleData_;
        data.oMg.resize(model.ngeoms);
        //data.activeCollisionPairs.resize(model.collisionPairs.size(), true)
        //data.distance_results(model.collisionPairs.size())
        //data.collision_results(model.collisionPairs.size())
        //data.radius()
        data.collisionObjects.push_back (fcl::CollisionObject(
              model.geometryObjects[id].fcl));
        data.oMg[id] =  model.geometryObjects[id].placement;
        data.collisionObjects[id].setTransform( se3::toFclTransform3f(data.oMg[id]) );
      }
      CollisionObjectPtr_t object (
          new CollisionObject(obstacleModel_,obstacleData_,id));

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
    }

    void ProblemSolver::removeObstacleFromJoint
    (const std::string& obstacleName, const std::string& jointName)
    {
      if (!robot_) {
	throw std::runtime_error ("No robot defined.");
      }
      JointPtr_t joint = robot_->getJointByName (jointName);
      if (!joint) {
	std::ostringstream oss;
	oss << "Robot has no joint with name " << jointName << ".";
	throw std::runtime_error (oss.str ().c_str ());
      }
      const CollisionObjectPtr_t& object = obstacle (obstacleName);
      if (!object)  {
	std::ostringstream oss;
	oss << "No obstacle with with name " << obstacleName << ".";
	throw std::runtime_error (oss.str ().c_str ());
      }
      problem ()->removeObstacleFromJoint (joint, object);
    }

    void ProblemSolver::filterCollisionPairs ()
    {
      problem()->filterCollisionPairs ();
    }

    CollisionObjectPtr_t ProblemSolver::obstacle (const std::string& name) const
    {
      if (obstacleModel_->existGeometryName(name)) {
        se3::GeomIndex id = obstacleModel_->getGeometryId(name);
        return CollisionObjectPtr_t (
            new CollisionObject(obstacleModel_,obstacleData_,id));
      }
      HPP_THROW(std::invalid_argument, "No obstacle with name " << name);
    }

    std::list <std::string> ProblemSolver::obstacleNames
    (bool collision, bool distance) const
    {
      std::list <std::string> res;
      if (collision) {
    for (ObjectStdVector_t::const_iterator it = collisionObstacles_.begin ();
	     it != collisionObstacles_.end (); ++it) {
	  res.push_back ((*it)->name ());
	}
      }
      if (distance) {
    for (ObjectStdVector_t::const_iterator it = distanceObstacles_.begin ();
	     it != distanceObstacles_.end (); ++it) {
	  res.push_back ((*it)->name ());
	}
      }
      return res;
    }

    const ObjectStdVector_t& ProblemSolver::collisionObstacles () const
    {
      return collisionObstacles_;
    }

    const ObjectStdVector_t& ProblemSolver::distanceObstacles () const
    {
      return distanceObstacles_;
    }

  } //   namespace core
} // namespace hpp
