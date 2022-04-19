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

#include <boost/serialization/weak_ptr.hpp>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/serialization.hh>
#include <hpp/util/timer.hh>
#include <limits>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace core {
using constraints::solver::BySubstitution;
typedef constraints::solver::lineSearch::Backtracking Backtracking_t;
typedef constraints::solver::lineSearch::ErrorNormBased ErrorNormBased_t;
typedef constraints::solver::lineSearch::FixedSequence FixedSequence_t;
typedef constraints::solver::lineSearch::Constant Constant_t;

ConfigProjector::LineSearchType ConfigProjector::defaultLineSearch_ =
    ConfigProjector::FixedSequence;

void ConfigProjector::defaultLineSearch(LineSearchType ls) {
  defaultLineSearch_ = ls;
}

HPP_DEFINE_REASON_FAILURE(REASON_MAX_ITER, "Max Iterations reached");
HPP_DEFINE_REASON_FAILURE(REASON_ERROR_INCREASED, "Error increased");
HPP_DEFINE_REASON_FAILURE(REASON_INFEASIBLE, "Problem infeasible");

ConfigProjectorPtr_t ConfigProjector::create(const DevicePtr_t& robot,
                                             const std::string& name,
                                             value_type errorThreshold,
                                             size_type maxIterations) {
  ConfigProjector* ptr =
      new ConfigProjector(robot, name, errorThreshold, maxIterations);
  ConfigProjectorPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConfigProjectorPtr_t ConfigProjector::createCopy(
    const ConfigProjectorPtr_t cp) {
  ConfigProjector* ptr = new ConfigProjector(*cp);
  ConfigProjectorPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConfigProjector::ConfigProjector(const DevicePtr_t& robot,
                                 const std::string& name,
                                 value_type _errorThreshold,
                                 size_type _maxIterations)
    : Constraint(name),
      robot_(robot),
      lineSearchType_(defaultLineSearch_),
      solver_(new BySubstitution(robot->configSpace()->vectorSpacesMerged())),
      weak_(),
      statistics_("ConfigProjector " + name) {
  errorThreshold(_errorThreshold);
  maxIterations(_maxIterations);
  lastIsOptional(false);
  solver_->saturation(
      make_shared<constraints::solver::saturation::Device>(robot_));
}

ConfigProjector::ConfigProjector(const ConfigProjector& cp)
    : Constraint(cp),
      robot_(cp.robot_),
      lineSearchType_(cp.lineSearchType_),
      solver_(new BySubstitution(*cp.solver_)),
      weak_(),
      statistics_(cp.statistics_) {}

ConfigProjector::~ConfigProjector() { delete solver_; }

ConstraintPtr_t ConfigProjector::copy() const {
  return createCopy(weak_.lock());
}

bool ConfigProjector::contains(
    const constraints::ImplicitPtr_t& numericalConstraint) const {
  return solver_->contains(numericalConstraint);
}

bool ConfigProjector::add(const constraints::ImplicitPtr_t& nm,
                          const std::size_t priority) {
  return solver_->add(nm, priority);
}

void ConfigProjector::computeValueAndJacobian(ConfigurationIn_t configuration,
                                              vectorOut_t value,
                                              matrixOut_t reducedJacobian) {
  Configuration_t q(configuration);
  // q_{out} = f (q_{in})
  solver_->explicitConstraintSet().solve(q);
  solver_->computeValue<true>(q);
  solver_->updateJacobian(q);  // includes the jacobian of the explicit system
  solver_->getValue(value);
  solver_->getReducedJacobian(reducedJacobian);
}

/// Convert vector of non locked degrees of freedom to vector of
/// all degrees of freedom
void ConfigProjector::uncompressVector(vectorIn_t small,
                                       vectorOut_t normal) const {
  solver_->explicitConstraintSet().notOutDers().transpose().lview(normal) =
      small;
}

void ConfigProjector::compressVector(vectorIn_t normal,
                                     vectorOut_t small) const {
  small =
      solver_->explicitConstraintSet().notOutDers().transpose().rview(normal);
}

void ConfigProjector::compressMatrix(matrixIn_t normal, matrixOut_t small,
                                     bool rows) const {
  if (rows) {
    typedef Eigen::MatrixBlockView<matrixIn_t, Eigen::Dynamic, Eigen::Dynamic,
                                   false, false>
        View;
    const Eigen::ColBlockIndices& cols =
        solver_->explicitConstraintSet().notOutDers();
    small = View(normal, cols.nbIndices(), cols.indices(), cols.nbIndices(),
                 cols.indices());
  } else {
    small = solver_->explicitConstraintSet().notOutDers().rview(normal);
  }
}

void ConfigProjector::uncompressMatrix(matrixIn_t small, matrixOut_t normal,
                                       bool rows) const {
  if (rows) {
    typedef Eigen::MatrixBlockView<matrixOut_t, Eigen::Dynamic, Eigen::Dynamic,
                                   false, false>
        View;
    const Eigen::ColBlockIndices& cols =
        solver_->explicitConstraintSet().notOutDers();
    View(normal, cols.nbIndices(), cols.indices(), cols.nbIndices(),
         cols.indices()) = small;
  } else {
    solver_->explicitConstraintSet().notOutDers().lview(normal) = small;
  }
}

bool ConfigProjector::impl_compute(ConfigurationOut_t configuration) {
  // If configuration satisfies the constraint, do not modify it
  if (isSatisfied(configuration)) return true;
  if (!(robot_->computationFlag() & pinocchio::JACOBIAN))
    throw std::runtime_error(
        "In ConfigProjector::apply: can't project a configuration if JACOBIAN "
        "computation flag is not enabled.");
  BySubstitution::Status status =
      (BySubstitution::Status)solverSolve(configuration);
  switch (status) {
    case BySubstitution::ERROR_INCREASED:
      statistics_.addFailure(REASON_ERROR_INCREASED);
      return false;
      break;
    case BySubstitution::MAX_ITERATION_REACHED:
      statistics_.addFailure(REASON_MAX_ITER);
      return false;
      break;
    case BySubstitution::INFEASIBLE:
      statistics_.addFailure(REASON_INFEASIBLE);
      return false;
      break;
    case BySubstitution::SUCCESS:
      statistics_.addSuccess();
      return true;
      break;
  }
  return false;
}

bool ConfigProjector::optimize(ConfigurationOut_t configuration,
                               std::size_t maxIter) {
  if (!lastIsOptional()) return true;
  if (!isSatisfied(configuration)) return false;
  const size_type maxIterSave = maxIterations();
  if (maxIter != 0) maxIterations(maxIter);
  hppDout(info, "before optimization: " << configuration.transpose());
  BySubstitution::Status status = BySubstitution::MAX_ITERATION_REACHED;
  switch (lineSearchType_) {
    case Backtracking: {
      Backtracking_t ls;
      status = solver_->solve(configuration, true, ls);
      break;
    }
    case ErrorNormBased: {
      ErrorNormBased_t ls;
      status = solver_->solve(configuration, true, ls);
      break;
    }
    case FixedSequence: {
      FixedSequence_t ls;
      status = solver_->solve(configuration, true, ls);
      break;
    }
    case Constant: {
      Constant_t ls;
      status = solver_->solve(configuration, true, ls);
      break;
    }
  }
  maxIterations(maxIterSave);
  hppDout(info, "After optimization: " << configuration.transpose());
  if (status == BySubstitution::SUCCESS)
    return true;
  else if (status == BySubstitution::INFEASIBLE && isSatisfied(configuration))
    return true;
  else {
    hppDout(info, "Optimization failed.");
    return false;
  }
}

void ConfigProjector::projectVectorOnKernel(ConfigurationIn_t from,
                                            vectorIn_t velocity,
                                            vectorOut_t result) {
  solver_->projectVectorOnKernel(from, velocity, result);
}

void ConfigProjector::projectOnKernel(ConfigurationIn_t from,
                                      ConfigurationIn_t to,
                                      ConfigurationOut_t result) {
  solver_->projectOnKernel(from, to, result);
}

std::ostream& ConfigProjector::print(std::ostream& os) const {
  return os << "Config projector: " << name() << ", contains " << *solver_
            << decindent;
}

bool ConfigProjector::isSatisfied(ConfigurationIn_t config) {
  return solver_->isSatisfied(config);
}

bool ConfigProjector::isSatisfied(ConfigurationIn_t config,
                                  value_type errorThreshold) {
  return solver_->isSatisfied(config, errorThreshold);
}

bool ConfigProjector::isSatisfied(ConfigurationIn_t config, vector_t& error) {
  error.resize(solver_->dimension() +
               solver_->explicitConstraintSet().outDers().nbIndices());
  return solver_->isSatisfied(config, error);
}

const NumericalConstraints_t& ConfigProjector::numericalConstraints() const {
  return solver_->numericalConstraints();
}

vector_t ConfigProjector::rightHandSideFromConfig(ConfigurationIn_t config) {
  return solver_->rightHandSideFromConfig(config);
}

void ConfigProjector::rightHandSideFromConfig(
    const constraints::ImplicitPtr_t& nm, ConfigurationIn_t config) {
  if (!solver_->rightHandSideFromConfig(nm, config)) {
    std::ostringstream os;
    os << "Function \"" << nm->function().name()
       << "\" was not found in the solver. Solver contains (";
    for (auto constraint : solver_->constraints()) {
      os << "\"" << constraint->function().name() << "\",";
    }
    os << ").";
    throw std::runtime_error(os.str().c_str());
  }
}

void ConfigProjector::rightHandSide(const vector_t& small) {
  solver_->rightHandSide(small);
}

void ConfigProjector::rightHandSide(const constraints::ImplicitPtr_t& nm,
                                    vectorIn_t rhs) {
  if (!solver_->rightHandSide(nm, rhs)) {
    std::ostringstream os;
    os << "Function \"" << nm->function().name()
       << "\" was not found in the solver. Solver contains (";
    for (auto constraint : solver_->constraints()) {
      os << "\"" << constraint->function().name() << "\",";
    }
    os << ").";
  }
}

vector_t ConfigProjector::rightHandSide() const {
  return solver_->rightHandSide();
}

void ConfigProjector::rightHandSideAt(const value_type& s) {
  solver_->rightHandSideAt(s);
}

inline bool ConfigProjector::solverOneStep(ConfigurationOut_t config) const {
  switch (lineSearchType_) {
    case Backtracking: {
      Backtracking_t ls;
      return solver_->oneStep(config, ls);
    }
    case ErrorNormBased: {
      ErrorNormBased_t ls;
      return solver_->oneStep(config, ls);
    }
    case FixedSequence: {
      FixedSequence_t ls;
      return solver_->oneStep(config, ls);
    }
    case Constant: {
      Constant_t ls;
      return solver_->oneStep(config, ls);
    }
  }
  return false;
}

inline int ConfigProjector::solverSolve(ConfigurationOut_t config) const {
  switch (lineSearchType_) {
    case Backtracking: {
      Backtracking_t ls;
      return solver_->solve(config, ls);
    }
    case ErrorNormBased: {
      ErrorNormBased_t ls;
      return solver_->solve(config, ls);
    }
    case FixedSequence: {
      FixedSequence_t ls;
      return solver_->solve(config, ls);
    }
    case Constant: {
      Constant_t ls;
      return solver_->solve(config, ls);
    }
  }
  throw std::runtime_error("Unknow line search type");
  return BySubstitution::MAX_ITERATION_REACHED;
}

void ConfigProjector::lastIsOptional(bool optional) {
  solver_->lastIsOptional(optional);
}

bool ConfigProjector::lastIsOptional() const {
  return solver_->lastIsOptional();
}

size_type ConfigProjector::numberFreeVariables() const {
  return solver_->numberFreeVariables();
}

size_type ConfigProjector::dimension() const {
  return solver_->reducedDimension();
}

void ConfigProjector::maxIterations(size_type iterations) {
  solver_->maxIterations(iterations);
}

size_type ConfigProjector::maxIterations() const {
  return solver_->maxIterations();
}

void ConfigProjector::errorThreshold(const value_type& threshold) {
  solver_->errorThreshold(threshold);
  solver_->inequalityThreshold(threshold);
}

value_type ConfigProjector::errorThreshold() const {
  return solver_->errorThreshold();
}

value_type ConfigProjector::residualError() const {
  return solver_->residualError();
}

const value_type& ConfigProjector::sigma() const { return solver_->sigma(); }

template <class Archive>
void ConfigProjector::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<Constraint>(*this));
  ar& BOOST_SERIALIZATION_NVP(robot_);
  ar& BOOST_SERIALIZATION_NVP(lineSearchType_);
  ar& BOOST_SERIALIZATION_NVP(solver_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
  if (!Archive::is_saving::value)
    statistics_.name_ = "ConfigProjector " + name();
}

HPP_SERIALIZATION_IMPLEMENT(ConfigProjector);
}  // namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT(hpp::core::ConfigProjector)
