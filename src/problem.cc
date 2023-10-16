//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011 CNRS
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

#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/debug.hh>
#include <iostream>

namespace hpp {
namespace core {
Container<ParameterDescription>& pds() {
  static Container<ParameterDescription> _parameterDescriptions;
  return _parameterDescriptions;
}

// ======================================================================

void Problem::declareParameter(const ParameterDescription& desc) {
  typedef Container<ParameterDescription> CPD_t;
  std::pair<CPD_t::iterator, bool> ret =
      pds().map.insert(CPD_t::value_type(desc.name(), desc));
  if (!ret.second) ret.first->second = desc;
}

// ======================================================================

const Container<ParameterDescription>& Problem::parameterDescriptions() {
  return pds();
}

// ======================================================================

const ParameterDescription& Problem::parameterDescription(
    const std::string& name) {
  if (pds().has(name)) return pds().get(name);
  throw std::invalid_argument("No parameter description with name " + name);
}

// ======================================================================

ProblemPtr_t Problem::create(DevicePtr_t robot) {
  ProblemPtr_t p(new Problem(robot));
  p->init(p);
  return p;
}

ProblemPtr_t Problem::createCopy(const ProblemConstPtr_t& other) {
  ProblemPtr_t p(new Problem(*other));
  p->init(p);
  return p;
}

// ======================================================================
Problem::Problem(DevicePtr_t robot)
    : robot_(robot), configValidations_(ConfigValidations::create()) {}

// ======================================================================

void Problem::init(ProblemWkPtr_t wkPtr) {
  wkPtr_ = wkPtr;

  distance_ = WeighedDistance::create(robot_);
  target_ = problemTarget::GoalConfigurations::create(wkPtr_.lock());
  steeringMethod_ = steeringMethod::Straight::create(wkPtr_.lock());
  pathValidation_ =
      pathValidation::createDiscretizedCollisionChecking(robot_, 0.05);
  configurationShooter_ = configurationShooter::Uniform::create(robot_);
  constraints(ConstraintSet::create(robot_, "default_constraint_set"));
}

// ======================================================================

Problem::Problem()
    : robot_(),
      distance_(),
      initConf_(),
      target_(),
      steeringMethod_(),
      configValidations_(),
      pathValidation_(),
      collisionObstacles_(),
      constraints_(),
      configurationShooter_() {
  assert(false && "This constructor should not be used.");
}

Problem::~Problem() {}

// ======================================================================

void Problem::initConfig(ConfigurationIn_t config) {
  initConf_ = config;
}

// ======================================================================

const Configurations_t Problem::goalConfigs() const {
  problemTarget::GoalConfigurationsPtr_t gc(
      HPP_DYNAMIC_PTR_CAST(problemTarget::GoalConfigurations, target_));
  if (gc) {
    return gc->configurations();
  }
  return Configurations_t();
}

// ======================================================================

void Problem::addGoalConfig(ConfigurationIn_t config) {
  problemTarget::GoalConfigurationsPtr_t gc(
      HPP_DYNAMIC_PTR_CAST(problemTarget::GoalConfigurations, target_));
  // If target is not an instance of GoalConfigurations, create new
  // instance
  if (!gc) {
    gc = problemTarget::GoalConfigurations::create(wkPtr_.lock());
    target_ = gc;
  }
  gc->addConfiguration(config);
}

// ======================================================================

void Problem::resetGoalConfigs() {
  problemTarget::GoalConfigurationsPtr_t gc(
      HPP_DYNAMIC_PTR_CAST(problemTarget::GoalConfigurations, target_));
  // If target is not an instance of GoalConfigurations, create new
  // instance
  if (!gc) {
    gc = problemTarget::GoalConfigurations::create(wkPtr_.lock());
    target_ = gc;
  }
  gc->resetConfigurations();
}

// ======================================================================

void Problem::clearConfigValidations() { configValidations_->clear(); }

// ======================================================================

const ObjectStdVector_t& Problem::collisionObstacles() const {
  return collisionObstacles_;
}

// ======================================================================

void Problem::collisionObstacles(const ObjectStdVector_t& collisionObstacles) {
  collisionObstacles_.clear();
  // pass the local vector of collisions object to the problem
  for (ObjectStdVector_t::const_iterator itObj = collisionObstacles.begin();
       itObj != collisionObstacles.end(); ++itObj) {
    addObstacle(*itObj);
  }
}

// ======================================================================

void Problem::addObstacle(const CollisionObjectPtr_t& object) {
  // Add object in local list
  collisionObstacles_.push_back(object);
  // Add obstacle to path validation method
  shared_ptr<ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, pathValidation_);
  if (oui) oui->addObstacle(object);
  assert(configValidations_);
  if (configValidations_) {
    configValidations_->addObstacle(object);
  }
}

// ======================================================================

void Problem::removeObstacleFromJoint(
    const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle) {
  shared_ptr<ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, pathValidation_);
  if (oui) oui->removeObstacleFromJoint(joint, obstacle);
  assert(configValidations_);
  if (configValidations_) {
    configValidations_->removeObstacleFromJoint(joint, obstacle);
  }
}

// ======================================================================

void Problem::filterCollisionPairs() {
  RelativeMotion::matrix_type matrix = RelativeMotion::matrix(robot_);
  RelativeMotion::fromConstraint(matrix, robot_, constraints_);
  hppDout(info, "RelativeMotion matrix:\n" << matrix);

  shared_ptr<ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, pathValidation_);
  if (oui) oui->filterCollisionPairs(matrix);
  assert(configValidations_);
  if (configValidations_) {
    configValidations_->filterCollisionPairs(matrix);
  }
}

// ======================================================================

void Problem::setSecurityMargins(const matrix_t& securityMatrix) {
  shared_ptr<ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, pathValidation_);
  if (oui) oui->setSecurityMargins(securityMatrix);
  assert(configValidations_);
  if (configValidations_) {
    configValidations_->setSecurityMargins(securityMatrix);
  }
}

// ======================================================================

void Problem::pathValidation(const PathValidationPtr_t& pathValidation) {
  pathValidation_ = pathValidation;
}

// ======================================================================

void Problem::addConfigValidation(
    const ConfigValidationPtr_t& configValidation) {
  configValidations_->add(configValidation);
}

// ======================================================================

void Problem::configurationShooter(
    const ConfigurationShooterPtr_t& configurationShooter) {
  configurationShooter_ = configurationShooter;
}

// ======================================================================

void Problem::checkProblem() const {
  if (!robot()) {
    std::string msg("No device in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  if (initConfig().size() == 0) {
    std::string msg("No init config in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  // Check that initial configuration is valid
  ValidationReportPtr_t report;
  if (!configValidations_->validate(initConf_, report)) {
    std::ostringstream oss;
    oss << "init config invalid : " << *report;
    throw std::runtime_error(oss.str());
  }
  // Check that initial configuration sarisfies the constraints of the
  // problem
  if (!constraints_->isSatisfied(initConf_)) {
    std::ostringstream os;
    os << "Initial configuration " << pinocchio::displayConfig(initConf_)
       << " does not satisfy the constraints of the problem.";
    throw std::logic_error(os.str().c_str());
  }

  // check that goal configurations satisfy are valid
  problemTarget::GoalConfigurationsPtr_t gc(
      HPP_DYNAMIC_PTR_CAST(problemTarget::GoalConfigurations, target_));
  if (gc) {
    const Configurations_t goals(gc->configurations());
    for (auto config : gc->configurations()) {
      if (!configValidations_->validate(config, report)) {
        std::ostringstream oss;
        oss << *report;
        throw std::runtime_error(oss.str());
      }
      // Check that goal configurations sarisfy the constraints of the
      // problem
      if (!constraints_->isSatisfied(config)) {
        std::ostringstream os;
        os << "Goal configuration " << pinocchio::displayConfig(config)
           << " does not satisfy the constraints of the problem.";
        throw std::logic_error(os.str().c_str());
      }
    }
  }
}
// ======================================================================

void Problem::setParameter(const std::string& name, const Parameter& value) {
  const ParameterDescription& desc = parameterDescription(name);
  if (desc.type() != value.type())
    throw std::invalid_argument("value is not a " +
                                Parameter::typeName(desc.type()));
  parameters.add(name, value);
}

// ======================================================================

}  // namespace core
}  // namespace hpp
