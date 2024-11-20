//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/fcl/collision_utility.h>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/configuration-shooter/gaussian.hh>
#include <hpp/core/configuration-shooter/uniform-tpl.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/diffusing-planner.hh>
#include <hpp/core/distance-between-objects.hh>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/kinodynamic-distance.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimization/rs-time-parameterization.hh>
#include <hpp/core/path-optimization/simple-shortcut.hh>
#include <hpp/core/path-optimization/simple-time-parameterization.hh>
#include <hpp/core/path-planner/bi-rrt-star.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-planner/search-in-roadmap.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem-target/task-target.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method/dubins.hh>
#include <hpp/core/steering-method/hermite.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/steering-method/snibud.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/steering-method/steering-kinodynamic.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/visibility-prm-planner.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <iterator>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include "../src/path-validation/no-validation.hh"

namespace hpp {
namespace core {
using pinocchio::GeomIndex;

using pinocchio::CollisionObject;
using pinocchio::Data;
using pinocchio::DataPtr_t;
using pinocchio::GeomData;
using pinocchio::GeomDataPtr_t;
using pinocchio::GeomModel;
using pinocchio::GeomModelPtr_t;
using pinocchio::Model;
using pinocchio::ModelPtr_t;

namespace {
template <typename Container>
void remove(Container& vector, std::size_t pos) {
  assert(pos < vector.size());
  vector.erase(std::next(vector.begin(), pos));
}

void remove(ObjectStdVector_t& vector, const GeomIndex& i) {
  ObjectStdVector_t::iterator it = std::find_if(
      vector.begin(), vector.end(),
      [i](const CollisionObjectPtr_t& co) { return co->indexInModel() == i; });
  if (it != vector.end()) vector.erase(it);
}
}  // namespace

pathValidation::DiscretizedPtr_t
createDiscretizedJointBoundAndCollisionChecking(const DevicePtr_t& robot,
                                                const value_type& stepSize) {
  using namespace pathValidation;
  return Discretized::create(stepSize, {
                                           JointBoundValidation::create(robot),
                                           CollisionValidation::create(robot),
                                       });
}

configurationShooter::GaussianPtr_t createGaussianConfigShooter(
    const ProblemConstPtr_t& p) {
  configurationShooter::GaussianPtr_t ptr(
      configurationShooter::Gaussian::create(p->robot()));
  static const std::string center = "ConfigurationShooter/Gaussian/center";
  static const std::string useVel =
      "ConfigurationShooter/Gaussian/useRobotVelocity";
  static const std::string stdDev =
      "ConfigurationShooter/Gaussian/standardDeviation";
  ptr->center(p->getParameter(center).vectorValue());
  if (p->getParameter(useVel).boolValue()) {
    ptr->sigmas(p->robot()->currentVelocity());
  } else if (p->parameters.has(stdDev))
    ptr->sigma(p->getParameter(stdDev).floatValue());
  return ptr;
}

configurationShooter::UniformPtr_t createUniformConfigShooter(
    const ProblemConstPtr_t& p) {
  configurationShooter::UniformPtr_t ptr(
      configurationShooter::Uniform::create(p->robot()));
  ptr->sampleExtraDOF(
      p->getParameter("ConfigurationShooter/sampleExtraDOF").boolValue());
  return ptr;
}

template <class generator_t>
typename configurationShooter::UniformTpl<generator_t>::Ptr_t createUniformTpl(
    const ProblemConstPtr_t& p) {
  typedef configurationShooter::UniformTpl<generator_t> shooter_t;
  typename shooter_t::Ptr_t ptr(shooter_t::create(p->robot()));
  return ptr;
}

ProblemSolverPtr_t ProblemSolver::create() {
  return ProblemSolverPtr_t(new ProblemSolver());
}

ProblemSolver::ProblemSolver()
    : constraints_(),
      robot_(),
      problem_(),
      pathPlanner_(),
      roadmap_(),
      paths_(),
      pathProjectorType_("None"),
      pathProjectorTolerance_(0.2),
      pathPlannerType_("DiffusingPlanner"),
      target_(problemTarget::GoalConfigurations::create(ProblemPtr_t())),
      initConf_(),
      goalConfigurations_(),
      robotType_("hpp::pinocchio::Device"),
      configurationShooterType_("Uniform"),
      distanceType_("WeighedDistance"),
      steeringMethodType_("Straight"),
      pathOptimizerTypes_(),
      pathOptimizers_(),
      pathValidationType_("Discretized"),
      pathValidationTolerance_(0.05),
      configValidationTypes_(),
      collisionObstacles_(),
      distanceObstacles_(),
      obstacleRModel_(new Model()),
      obstacleRData_(),
      obstacleModel_(new GeomModel()),
      obstacleData_(new GeomData(*obstacleModel_)),
      errorThreshold_(1e-4),
      maxIterProjection_(20),
      maxIterPathPlanning_(std::numeric_limits<unsigned long int>::max()),
      timeOutPathPlanning_(std::numeric_limits<double>::infinity()),

      passiveDofsMap_(),
      comcMap_(),
      distanceBetweenObjects_() {
  obstacleRModel_->addFrame(::pinocchio::Frame(
      "obstacle_frame", 0, 0, Transform3f::Identity(), ::pinocchio::BODY));
  obstacleRData_.reset(new Data(*obstacleRModel_));

  robots.add(robotType_, Device_t::create);
  pathPlanners.add("DiffusingPlanner", DiffusingPlanner::createWithRoadmap);
  pathPlanners.add("VisibilityPrmPlanner",
                   VisibilityPrmPlanner::createWithRoadmap);
  pathPlanners.add("BiRRTPlanner", BiRRTPlanner::createWithRoadmap);
  pathPlanners.add("kPRM*", pathPlanner::kPrmStar::createWithRoadmap);
  pathPlanners.add("BiRRT*", pathPlanner::BiRrtStar::createWithRoadmap);
  pathPlanners.add("SearchInRoadmap", hpp::core::pathPlanner::SearchInRoadmap::createWithRoadmap);

  configurationShooters.add("Uniform", createUniformConfigShooter);
  configurationShooters.add("Gaussian", createGaussianConfigShooter);
  configurationShooters.add("UniformSeedable",
                            createUniformTpl<std::default_random_engine>);

  distances.add("Weighed", WeighedDistance::createFromProblem);
  distances.add(
      "ReedsShepp",
      std::bind(
          static_cast<distance::ReedsSheppPtr_t (*)(const ProblemConstPtr_t&)>(
              distance::ReedsShepp::create),
          std::placeholders::_1));
  distances.add("Kinodynamic", KinodynamicDistance::createFromProblem);

  steeringMethods.add("Straight", steeringMethod::Straight::create);
  steeringMethods.add("ReedsShepp",
                      steeringMethod::ReedsShepp::createWithGuess);
  steeringMethods.add("Kinodynamic", steeringMethod::Kinodynamic::create);
  steeringMethods.add("Dubins", steeringMethod::Dubins::createWithGuess);
  steeringMethods.add("Snibud", steeringMethod::Snibud::createWithGuess);
  steeringMethods.add("Hermite", steeringMethod::Hermite::create);
  steeringMethods.add("SplineBezier3",
                      steeringMethod::Spline<path::BernsteinBasis, 3>::create);
  steeringMethods.add("SplineBezier5",
                      steeringMethod::Spline<path::BernsteinBasis, 5>::create);

  // Store path optimization methods in map.
  pathOptimizers.add("RandomShortcut",
                     pathOptimization::RandomShortcut::create);
  pathOptimizers.add("SimpleShortcut",
                     pathOptimization::SimpleShortcut::create);
  pathOptimizers.add("PartialShortcut",
                     pathOptimization::PartialShortcut::create);
  pathOptimizers.add("SimpleTimeParameterization",
                     pathOptimization::SimpleTimeParameterization::create);
  pathOptimizers.add("RSTimeParameterization",
                     pathOptimization::RSTimeParameterization::create);

  // Store path validation methods in map.
  pathValidations.add("NoValidation", pathValidation::NoValidation::create);
  pathValidations.add("Discretized",
                      pathValidation::createDiscretizedCollisionChecking);
  pathValidations.add("DiscretizedCollision",
                      pathValidation::createDiscretizedCollisionChecking);
  pathValidations.add("DiscretizedJointBound",
                      pathValidation::createDiscretizedJointBound);
  pathValidations.add("DiscretizedCollisionAndJointBound",
                      createDiscretizedJointBoundAndCollisionChecking);
  pathValidations.add("Progressive", continuousValidation::Progressive::create);
  pathValidations.add("Dichotomy", continuousValidation::Dichotomy::create);

  // Store config validation methods in map.
  configValidations.add("CollisionValidation", CollisionValidation::create);
  configValidations.add("JointBoundValidation", JointBoundValidation::create);

  // Set default config validation methods.
  configValidationTypes_.push_back("CollisionValidation");
  configValidationTypes_.push_back("JointBoundValidation");

  // Store path projector methods in map.
  pathProjectors.add("None", [](const ProblemConstPtr_t&, const value_type&) {
    return PathProjectorPtr_t();
  });
  pathProjectors.add("Progressive",
                     [](const ProblemConstPtr_t& p, const value_type& v) {
                       return pathProjector::Progressive ::create(p, v);
                     });
  pathProjectors.add("Dichotomy",
                     [](const ProblemConstPtr_t& p, const value_type& v) {
                       return pathProjector::Dichotomy ::create(p, v);
                     });
  pathProjectors.add("Global",
                     [](const ProblemConstPtr_t& p, const value_type& v) {
                       return pathProjector::Global ::create(p, v);
                     });
  pathProjectors.add("RecursiveHermite",
                     [](const ProblemConstPtr_t& p, const value_type& v) {
                       return pathProjector::RecursiveHermite::create(p, v);
                     });
}

ProblemSolver::~ProblemSolver() {}

void ProblemSolver::distanceType(const std::string& type) {
  if (!distances.has(type)) {
    throw std::runtime_error(std::string("No distance method with name ") +
                             type);
  }
  distanceType_ = type;
  // TODO
  // initDistance ();
}

void ProblemSolver::steeringMethodType(const std::string& type) {
  if (!steeringMethods.has(type)) {
    throw std::runtime_error(std::string("No steering method with name ") +
                             type);
  }
  steeringMethodType_ = type;
  if (problem_) initSteeringMethod();
}

void ProblemSolver::pathPlannerType(const std::string& type) {
  if (!pathPlanners.has(type)) {
    throw std::runtime_error(std::string("No path planner with name ") + type);
  }
  pathPlannerType_ = type;
}

void ProblemSolver::configurationShooterType(const std::string& type) {
  if (!configurationShooters.has(type)) {
    throw std::runtime_error(
        std::string("No configuration shooter with name ") + type);
  }
  configurationShooterType_ = type;
  if (robot_ && problem_) {
    problem_->configurationShooter(
        configurationShooters.get(configurationShooterType_)(problem_));
  }
}

void ProblemSolver::addPathOptimizer(const std::string& type) {
  if (!pathOptimizers.has(type)) {
    throw std::runtime_error(std::string("No path optimizer with name ") +
                             type);
  }
  pathOptimizerTypes_.push_back(type);
}

void ProblemSolver::clearPathOptimizers() {
  pathOptimizerTypes_.clear();
  pathOptimizers_.clear();
}

void ProblemSolver::optimizePath(PathVectorPtr_t path) {
  createPathOptimizers();
  for (PathOptimizers_t::const_iterator it = pathOptimizers_.begin();
       it != pathOptimizers_.end(); ++it) {
    path = (*it)->optimize(path);
    paths_.push_back(path);
  }
}

void ProblemSolver::pathValidationType(const std::string& type,
                                       const value_type& tolerance) {
  if (!pathValidations.has(type)) {
    throw std::runtime_error(std::string("No path validation method with "
                                         "name ") +
                             type);
  }
  pathValidationType_ = type;
  pathValidationTolerance_ = tolerance;
  // If a robot is present, set path validation method
  if (robot_ && problem_) {
    initPathValidation();
  }
}

// Initialize path validation with obstacles of problem
static void setObstaclesToPathValidation(const ProblemPtr_t& problem,
                                         const PathValidationPtr_t& pv) {
  // Insert obstacles in path validation object
  shared_ptr<ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, pv);
  if (oui) {
    for (ObjectStdVector_t::const_iterator it =
             problem->collisionObstacles().begin();
         it != problem->collisionObstacles().end(); ++it)
      oui->addObstacle(*it);
  }
}

void ProblemSolver::initPathValidation() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  PathValidationPtr_t pathValidation = pathValidations.get(pathValidationType_)(
      robot_, pathValidationTolerance_);
  setObstaclesToPathValidation(problem_, pathValidation);
  problem_->pathValidation(pathValidation);
}

void ProblemSolver::initConfigValidation() {
  if (!robot_) throw std::logic_error("You must provide a robot first");
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  problem_->clearConfigValidations();
  for (ConfigValidationTypes_t::const_iterator it =
           configValidationTypes_.begin();
       it != configValidationTypes_.end(); ++it) {
    ConfigValidationPtr_t configValidation = configValidations.get(*it)(robot_);
    problem_->addConfigValidation(configValidation);
  }
}

void ProblemSolver::initValidations() {
  initPathValidation();
  initConfigValidation();
  problem_->collisionObstacles(collisionObstacles_);
}

void ProblemSolver::pathProjectorType(const std::string& type,
                                      const value_type& tolerance) {
  if (!pathProjectors.has(type)) {
    throw std::runtime_error(std::string("No path projector method with "
                                         "name ") +
                             type);
  }
  pathProjectorType_ = type;
  pathProjectorTolerance_ = tolerance;
  // If a robot is present, set path projector method
  if (robot_ && problem_) {
    initPathProjector();
  }
}

void ProblemSolver::addConfigValidationBuilder(
    const std::string& type, const ConfigValidationBuilder_t& builder) {
  configValidations.add(type, builder);
}

void ProblemSolver::addConfigValidation(const std::string& type) {
  if (!configValidations.has(type)) {
    throw std::runtime_error(std::string("No config validation method with "
                                         "name ") +
                             type);
  }
  configValidationTypes_.push_back(type);
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  // If a robot is present, set config validation methods
  initConfigValidation();
  // Set obstacles
  problem_->collisionObstacles(collisionObstacles_);
}

void ProblemSolver::clearConfigValidations() {
  configValidationTypes_.clear();
  problem_->clearConfigValidations();
}

void ProblemSolver::robotType(const std::string& type) { robotType_ = type; }

const std::string& ProblemSolver::robotType() const { return robotType_; }

DevicePtr_t ProblemSolver::createRobot(const std::string& name) {
  RobotBuilder_t factory(robots.get(robotType_));
  assert(factory);
  return factory(name);
}

void ProblemSolver::robot(const DevicePtr_t& robot) {
  robot_ = robot;
  constraints_ = ConstraintSet::create(robot_, "Default constraint set");
  // Reset obstacles
  obstacleRModel_.reset(new Model());
  obstacleRModel_->addFrame(::pinocchio::Frame(
      "obstacle_frame", 0, 0, Transform3f::Identity(), ::pinocchio::BODY));
  obstacleRData_.reset(new Data(*obstacleRModel_));
  obstacleModel_.reset(new GeomModel());
  obstacleData_.reset(new GeomData(*obstacleModel_));
  resetProblem();
}

const DevicePtr_t& ProblemSolver::robot() const { return robot_; }

void ProblemSolver::initConfig(ConfigurationIn_t config) { initConf_ = config; }

const Configurations_t& ProblemSolver::goalConfigs() const {
  return goalConfigurations_;
}

void ProblemSolver::addGoalConfig(ConfigurationIn_t config) {
  goalConfigurations_.push_back(config);
}

void ProblemSolver::resetGoalConfigs() { goalConfigurations_.clear(); }

void ProblemSolver::setGoalConstraints(
    const NumericalConstraints_t& constraints) {
  problemTarget::TaskTargetPtr_t tt(
      problemTarget::TaskTarget::create(problem_));
  ConstraintSetPtr_t cs(
      ConstraintSet::create(problem_->robot(), "goalConstraints"));
  ConfigProjectorPtr_t cp(ConfigProjector::create(
      robot_, "Goal ConfigProjector", errorThreshold_, maxIterProjection_));
  cs->addConstraint(cp);
  for (auto c : constraints) {
    cp->add(c);
  }
  tt->constraints(cs);
  target_ = tt;
}

void ProblemSolver::resetGoalConstraints() { target_.reset(); }

void ProblemSolver::addConstraint(const ConstraintPtr_t& constraint) {
  if (robot_)
    constraints_->addConstraint(constraint);
  else
    throw std::logic_error("Cannot add constraint while robot is not set");
}

void ProblemSolver::resetConstraints() {
  if (robot_) {
    constraints_ = ConstraintSet::create(robot_, "Default constraint set");
    if (problem_) {
      problem_->constraints(constraints_);
    }
  }
}

void ProblemSolver::addNumericalConstraintToConfigProjector(
    const std::string& configProjName, const std::string& constraintName,
    const std::size_t priority) {
  if (!robot_) {
    hppDout(error, "Cannot add constraint while robot is not set");
  }
  ConfigProjectorPtr_t configProjector = constraints_->configProjector();
  if (!configProjector) {
    configProjector = ConfigProjector::create(
        robot_, configProjName, errorThreshold_, maxIterProjection_);
    constraints_->addConstraint(configProjector);
  }
  if (!numericalConstraints.has(constraintName)) {
    std::stringstream ss;
    ss << "Function " << constraintName << " does not exists";
    throw std::invalid_argument(ss.str());
  }
  configProjector->add(numericalConstraints.get(constraintName), priority);
}

void ProblemSolver::comparisonType(const std::string& name,
                                   const ComparisonTypes_t types) {
  constraints::ImplicitPtr_t nc;
  if (numericalConstraints.has(name))
    nc = numericalConstraints.get(name);
  else
    throw std::runtime_error(name + std::string(" is not a numerical "
                                                "constraint."));
  nc->comparisonType(types);
}

void ProblemSolver::comparisonType(const std::string& name,
                                   const ComparisonType& type) {
  constraints::ImplicitPtr_t nc;
  if (numericalConstraints.has(name))
    nc = numericalConstraints.get(name);
  else
    throw std::runtime_error(name + std::string(" is not a numerical "
                                                "constraint."));
  ComparisonTypes_t eqtypes(nc->function().outputDerivativeSize(), type);
  nc->comparisonType(eqtypes);
}

ComparisonTypes_t ProblemSolver::comparisonType(const std::string& name) const {
  constraints::ImplicitPtr_t nc;
  if (numericalConstraints.has(name))
    nc = numericalConstraints.get(name);
  else
    throw std::runtime_error(name + std::string(" is not a numerical "
                                                "constraint."));
  return nc->comparisonType();
}

void ProblemSolver::computeValueAndJacobian(
    const Configuration_t& configuration, vector_t& value,
    matrix_t& jacobian) const {
  if (!robot()) throw std::runtime_error("No robot loaded");
  ConfigProjectorPtr_t configProjector(constraints()->configProjector());
  if (!configProjector) {
    throw std::runtime_error("No constraints have assigned.");
  }
  // resize value and Jacobian
  value.resize(configProjector->solver().dimension());
  size_type rows = configProjector->solver().reducedDimension();
  jacobian.resize(rows, configProjector->numberFreeVariables());
  configProjector->computeValueAndJacobian(configuration, value, jacobian);
}

void ProblemSolver::maxIterProjection(size_type iterations) {
  maxIterProjection_ = iterations;
  if (constraints_ && constraints_->configProjector()) {
    constraints_->configProjector()->maxIterations(iterations);
  }
}

void ProblemSolver::maxIterPathPlanning(size_type iterations) {
  maxIterPathPlanning_ = iterations;
  if (pathPlanner_) pathPlanner_->maxIterations(maxIterPathPlanning_);
}

void ProblemSolver::errorThreshold(const value_type& threshold) {
  errorThreshold_ = threshold;
  if (constraints_ && constraints_->configProjector()) {
    constraints_->configProjector()->errorThreshold(threshold);
  }
}

void ProblemSolver::resetProblem() {
  initializeProblem(Problem::create(robot_));
}

void ProblemSolver::initializeProblem(ProblemPtr_t problem) {
  problem_ = problem;
  resetRoadmap();
  // Set constraints
  problem_->constraints(constraints_);
  // Set path validation method
  initPathValidation();
  // Set config validation methods
  initConfigValidation();
  // Set obstacles
  problem_->collisionObstacles(collisionObstacles_);
  // Distance to obstacles
  distanceBetweenObjects_ =
      DistanceBetweenObjectsPtr_t(new DistanceBetweenObjects(robot_));
  distanceBetweenObjects_->obstacles(distanceObstacles_);
}

void ProblemSolver::initProblem() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  // Set shooter
  problem_->configurationShooter(
      configurationShooters.get(configurationShooterType_)(problem_));
  // Set steeringMethod
  initSteeringMethod();
  PathPlannerBuilder_t createPlanner = pathPlanners.get(pathPlannerType_);
  pathPlanner_ = createPlanner(problem_, roadmap_);
  pathPlanner_->maxIterations(maxIterPathPlanning_);
  pathPlanner_->timeOut(timeOutPathPlanning_);
  roadmap_ = pathPlanner_->roadmap();
  /// create Path projector
  initPathProjector();
  /// create Path optimizer
  // Reset init and goal configurations
  problem_->initConfig(initConf_);
  problemTarget::TaskTargetPtr_t tt(
      HPP_DYNAMIC_PTR_CAST(problemTarget::TaskTarget, target_));
  if (!goalConfigurations_.empty() && tt) {
    throw std::logic_error(
        "The goal is defined by goal configurations and"
        " by a task at the same time. This is not"
        " valid.");
  }
  if (!tt) {
    // Create a goal with configurations
    problemTarget::GoalConfigurationsPtr_t gc(
        problemTarget::GoalConfigurations::create(problem_));
    for (Configurations_t::const_iterator itConfig =
             goalConfigurations_.begin();
         itConfig != goalConfigurations_.end(); ++itConfig) {
      gc->addConfiguration(*itConfig);
    }
    target_ = gc;
  }
  initProblemTarget();
}

void ProblemSolver::problem(ProblemPtr_t problem) { problem_ = problem; }

void ProblemSolver::resetRoadmap() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  roadmap_ = Roadmap::create(problem_->distance(), problem_->robot());
}

void ProblemSolver::createPathOptimizers() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  pathOptimizers_.clear();
  for (PathOptimizerTypes_t::const_iterator it = pathOptimizerTypes_.begin();
       it != pathOptimizerTypes_.end(); ++it) {
    PathOptimizerBuilder_t createOptimizer = pathOptimizers.get(*it);
    pathOptimizers_.push_back(createOptimizer(problem_));
  }
}

void ProblemSolver::initDistance() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  DistancePtr_t dist(distances.get(distanceType_)(problem_));
  problem_->distance(dist);
}

void ProblemSolver::initSteeringMethod() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  SteeringMethodPtr_t sm(steeringMethods.get(steeringMethodType_)(problem_));
  problem_->steeringMethod(sm);
}

void ProblemSolver::initPathProjector() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  PathProjectorBuilder_t createProjector =
      pathProjectors.get(pathProjectorType_);
  // The PathProjector will store a copy of the current steering method.
  // This means:
  // - when constraints are relevant, they should have been added before.
  //   TODO The path projector should update the constraint according to the
  //   path they project.
  // - the steering method type must match the path projector type.
  PathProjectorPtr_t pathProjector_ =
      createProjector(problem_, pathProjectorTolerance_);
  problem_->pathProjector(pathProjector_);
}

void ProblemSolver::initProblemTarget() {
  if (!problem_) throw std::runtime_error("The problem is not defined.");
  target_->problem(problem_);
  problem_->target(target_);
}

bool ProblemSolver::prepareSolveStepByStep() {
  initProblem();

  pathPlanner_->startSolve();
  pathPlanner_->tryConnectInitAndGoals();
  return roadmap_->pathExists();
}

bool ProblemSolver::executeOneStep() {
  pathPlanner_->oneStep();
  return roadmap_->pathExists();
}

void ProblemSolver::finishSolveStepByStep() {
  if (!roadmap_->pathExists()) throw std::logic_error("No path exists.");
  PathVectorPtr_t planned = pathPlanner_->computePath();
  paths_.push_back(pathPlanner_->finishSolve(planned));
}

void ProblemSolver::solve() {
  initProblem();

  PathVectorPtr_t path = pathPlanner_->solve();
  paths_.push_back(path);
  optimizePath(path);
}

bool ProblemSolver::directPath(ConfigurationIn_t start, ConfigurationIn_t end,
                               bool validate, std::size_t& pathId,
                               std::string& report) {
  report = "";
  if (!problem_) throw std::runtime_error("The problem is not defined.");

  // Get steering method from problem
  SteeringMethodPtr_t sm(problem_->steeringMethod());
  PathPtr_t dp = (*sm)(start, end);
  if (!dp) {
    report = "Steering method failed to build a path.";
    pathId = -1;
    return false;
  }
  PathPtr_t dp1, dp2;
  PathValidationReportPtr_t r;
  bool projValid = true, projected = true, pathValid = true;
  if (validate) {
    PathProjectorPtr_t proj(problem()->pathProjector());
    if (proj) {
      projected = proj->apply(dp, dp1);
    } else {
      dp1 = dp;
    }
    projValid = problem()->pathValidation()->validate(dp1, false, dp2, r);
    pathValid = projValid && projected;
    if (!projValid) {
      if (r) {
        std::ostringstream oss;
        oss << *r;
        report = oss.str();
      } else {
        report = "No path validation report.";
      }
    }
  } else {
    dp2 = dp;
  }
  // Add Path in problem
  PathVectorPtr_t path(
      core::PathVector::create(dp2->outputSize(), dp2->outputDerivativeSize()));
  path->appendPath(dp2);
  pathId = addPath(path);
  return pathValid;
}

void ProblemSolver::addConfigToRoadmap(ConfigurationIn_t config) {
  roadmap_->addNode(config);
}

void ProblemSolver::addEdgeToRoadmap(ConfigurationIn_t config1,
                                     ConfigurationIn_t config2,
                                     const PathPtr_t& path) {
  NodePtr_t node1, node2;
  value_type accuracy = 10e-6;
  value_type distance1, distance2;
  node1 = roadmap_->nearestNode(config1, distance1);
  node2 = roadmap_->nearestNode(config2, distance2);
  if (distance1 >= accuracy) {
    throw std::runtime_error("No node of the roadmap contains config1");
  }
  if (distance2 >= accuracy) {
    throw std::runtime_error("No node of the roadmap contains config2");
  }
  roadmap_->addEdge(node1, node2, path);
}

void ProblemSolver::interrupt() {
  if (pathPlanner()) pathPlanner()->interrupt();
  for (PathOptimizers_t::iterator it = pathOptimizers_.begin();
       it != pathOptimizers_.end(); ++it) {
    (*it)->interrupt();
  }
}

void ProblemSolver::addObstacle(const DevicePtr_t& device, bool collision,
                                bool distance) {
  device->computeForwardKinematics(pinocchio::JOINT_POSITION);
  device->computeFramesForwardKinematics();
  device->updateGeometryPlacements();
  const std::string& prefix = device->name();

  const Model& m = device->model();
  const Data& d = device->data();
  for (std::size_t i = 1; i < m.frames.size(); ++i) {
    const ::pinocchio::Frame& frame = m.frames[i];
    // ::pinocchio::FrameType type = (frame.type == ::pinocchio::JOINT ?
    // ::pinocchio::FIXED_JOINT : frame.type);
    obstacleRModel_->addFrame(
        ::pinocchio::Frame(prefix + frame.name, 0,
                           0,  // TODO Keep frame hierarchy
                           d.oMf[i],
                           // type
                           frame.type));
  }
  obstacleRData_.reset(new Data(*obstacleRModel_));
  ::pinocchio::framesForwardKinematics(*obstacleRModel_, *obstacleRData_,
                                       vector_t::Zero(0).eval());

  // Detach objects from joints
  for (size_type i = 0; i < device->nbObjects(); ++i) {
    CollisionObjectPtr_t obj = device->objectAt(i);
    addObstacle(prefix + obj->name(), obj->geometry(), obj->getTransform(),
                collision, distance);
  }
}

void ProblemSolver::addObstacle(const CollisionObjectPtr_t& object,
                                bool collision, bool distance) {
  // FIXME propagate object->mesh_path ?
  addObstacle(object->name(), object->geometry(), object->getTransform(),
              collision, distance);
}

void ProblemSolver::addObstacle(const std::string& name,
                                const CollisionGeometryPtr_t& inObject,
                                const Transform3f& pose, bool collision,
                                bool distance) {
  if (obstacleModel_->existGeometryName(name)) {
    HPP_THROW(std::runtime_error,
              "object with name "
                  << name << " already added! Choose another name (prefix).");
  }

  ::pinocchio::GeomIndex id = obstacleModel_->addGeometryObject(
      ::pinocchio::GeometryObject(name, 1, 0, inObject, pose, "",
                                  vector3_t::Ones()),
      *obstacleRModel_);
  // Update obstacleData_
  // FIXME This should be done in Pinocchio
  {
    ::pinocchio::GeometryModel& model = *obstacleModel_;
    ::pinocchio::GeometryData& data = *obstacleData_;
    data.oMg.resize(model.ngeoms);
    // data.activeCollisionPairs.resize(model.collisionPairs.size(), true)
    // data.distance_results(model.collisionPairs.size())
    // data.collision_results(model.collisionPairs.size())
    // data.radius()
    data.oMg[id] = model.geometryObjects[id].placement;
  }
  CollisionObjectPtr_t object(
      new CollisionObject(obstacleModel_, obstacleData_, id));

  if (collision) {
    collisionObstacles_.push_back(object);
    resetRoadmap();
  }
  if (distance) distanceObstacles_.push_back(object);
  if (problem()) problem()->addObstacle(object);
  if (distanceBetweenObjects_) {
    distanceBetweenObjects_->addObstacle(object);
  }
}

void ProblemSolver::addObstacle(const std::string& name,
                                /*const*/ FclCollisionObject& inObject,
                                bool collision, bool distance) {
  if (obstacleModel_->existGeometryName(name)) {
    HPP_THROW(std::runtime_error,
              "object with name "
                  << name << " already added! Choose another name (prefix).");
  }

  ::pinocchio::GeomIndex id = obstacleModel_->addGeometryObject(
      ::pinocchio::GeometryObject(
          name, 1, 0, inObject.collisionGeometry(),
          ::pinocchio::toPinocchioSE3(inObject.getTransform()), "",
          vector3_t::Ones()),
      *obstacleRModel_);
  // Update obstacleData_
  // FIXME This should be done in Pinocchio
  {
    ::pinocchio::GeometryModel& model = *obstacleModel_;
    ::pinocchio::GeometryData& data = *obstacleData_;
    data.oMg.resize(model.ngeoms);
    // data.activeCollisionPairs.resize(model.collisionPairs.size(), true)
    // data.distance_results(model.collisionPairs.size())
    // data.collision_results(model.collisionPairs.size())
    // data.radius()
    data.oMg[id] = model.geometryObjects[id].placement;
  }
  CollisionObjectPtr_t object(
      new CollisionObject(obstacleModel_, obstacleData_, id));

  if (collision) {
    collisionObstacles_.push_back(object);
    resetRoadmap();
  }
  if (distance) distanceObstacles_.push_back(object);
  if (problem()) problem()->addObstacle(object);
  if (distanceBetweenObjects_) {
    distanceBetweenObjects_->addObstacle(object);
  }
}

void ProblemSolver::removeObstacle(const std::string& name) {
  if (!obstacleModel_->existGeometryName(name)) {
    HPP_THROW(std::invalid_argument, "No obstacle with name " << name);
  }
  ::pinocchio::GeomIndex id = obstacleModel_->getGeometryId(name);

  // Update obstacle model
  remove(obstacleModel_->geometryObjects, id);
  obstacleModel_->ngeoms--;
  remove(obstacleData_->oMg, id);

  remove(collisionObstacles_, id);
  remove(distanceObstacles_, id);
  for (ObjectStdVector_t::iterator _o = collisionObstacles_.begin();
       _o != collisionObstacles_.end(); ++_o) {
    pinocchio::GeomIndex oid = (*_o)->indexInModel();
    if (oid > id)
      _o->reset(new CollisionObject(obstacleModel_, obstacleData_, oid - 1));
  }
  for (ObjectStdVector_t::iterator _o = distanceObstacles_.begin();
       _o != distanceObstacles_.end(); ++_o) {
    pinocchio::GeomIndex oid = (*_o)->indexInModel();
    if (oid > id)
      _o->reset(new CollisionObject(obstacleModel_, obstacleData_, oid - 1));
  }
  resetProblem();  // resets problem_ and distanceBetweenObjects_
  resetRoadmap();
}

void ProblemSolver::cutObstacle(const std::string& name,
                                const fcl::AABB& aabb) {
  if (!obstacleModel_->existGeometryName(name)) {
    HPP_THROW(std::invalid_argument, "No obstacle with name " << name);
  }
  ::pinocchio::GeomIndex id = obstacleModel_->getGeometryId(name);

  fcl::Transform3f oMg = ::pinocchio::toFclTransform3f(obstacleData_->oMg[id]);
  CollisionGeometryPtr_t fclgeom = obstacleModel_->geometryObjects[id].geometry;
  CollisionGeometryPtr_t newgeom(extract(fclgeom.get(), oMg, aabb));
  if (!newgeom) {
    // No intersection. Geom should be removed.
    removeObstacle(name);
  } else {
    obstacleModel_->geometryObjects[id].geometry = newgeom;
  }
}

void ProblemSolver::removeObstacleFromJoint(const std::string& obstacleName,
                                            const std::string& jointName) {
  if (!robot_) {
    throw std::runtime_error("No robot defined.");
  }
  JointPtr_t joint = robot_->getJointByName(jointName);
  if (!joint) {
    std::ostringstream oss;
    oss << "Robot has no joint with name " << jointName << ".";
    throw std::runtime_error(oss.str().c_str());
  }
  const CollisionObjectPtr_t& object = obstacle(obstacleName);
  if (!object) {
    std::ostringstream oss;
    oss << "No obstacle with with name " << obstacleName << ".";
    throw std::runtime_error(oss.str().c_str());
  }
  problem()->removeObstacleFromJoint(joint, object);
}

void ProblemSolver::filterCollisionPairs() {
  problem()->filterCollisionPairs();
}

CollisionObjectPtr_t ProblemSolver::obstacle(const std::string& name) const {
  if (obstacleModel_->existGeometryName(name)) {
    ::pinocchio::GeomIndex id = obstacleModel_->getGeometryId(name);
    return CollisionObjectPtr_t(
        new CollisionObject(obstacleModel_, obstacleData_, id));
  }
  HPP_THROW(std::invalid_argument, "No obstacle with name " << name);
}

const Transform3f& ProblemSolver::obstacleFramePosition(
    const std::string& name) const {
  if (!obstacleRModel_->existFrame(name)) {
    HPP_THROW(std::invalid_argument, "No obstacle frame with name " << name);
  }
  ::pinocchio::FrameIndex id = obstacleRModel_->getFrameId(name);
  return obstacleRData_->oMf[id];
}

std::list<std::string> ProblemSolver::obstacleNames(bool collision,
                                                    bool distance) const {
  std::list<std::string> res;
  if (collision) {
    for (ObjectStdVector_t::const_iterator it = collisionObstacles_.begin();
         it != collisionObstacles_.end(); ++it) {
      res.push_back((*it)->name());
    }
  }
  if (distance) {
    for (ObjectStdVector_t::const_iterator it = distanceObstacles_.begin();
         it != distanceObstacles_.end(); ++it) {
      res.push_back((*it)->name());
    }
  }
  return res;
}

const ObjectStdVector_t& ProblemSolver::collisionObstacles() const {
  return collisionObstacles_;
}

const ObjectStdVector_t& ProblemSolver::distanceObstacles() const {
  return distanceObstacles_;
}

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(ProblemSolver)
Problem::declareParameter(ParameterDescription(
    Parameter::VECTOR, "ConfigurationShooter/Gaussian/center",
    "Center of gaussian random distribution.", Parameter(vector_t())));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "ConfigurationShooter/Gaussian/useRobotVelocity",
    "Use robot current velocity as standard velocity.", Parameter(false)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "ConfigurationShooter/Gaussian/standardDeviation",
    "Scale the default standard deviation with this factor.", Parameter(0.25)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "ConfigurationShooter/sampleExtraDOF",
    "If false, the value of the random configuration extraDOF are set to 0.",
    Parameter(true)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "SteeringMethod/Carlike/turningRadius",
    "Turning radius of a carlike robot for Reeds and Shepp and Dubins "
    "distances and steering methods.",
    Parameter(1.)));
Problem::declareParameter(ParameterDescription(
    Parameter::STRING, "SteeringMethod/Carlike/wheels",
    "Names of revolute joints that hold directional wheels separated by "
    "commas.",
    Parameter(std::string(""))));
HPP_END_PARAMETER_DECLARATION(ProblemSolver)
}  //   namespace core
}  // namespace hpp
