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

#include <hpp/core/config-projector.hh>

#include <limits>

#include <boost/bind.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/constraint-set.hh>
// #include <hpp/constraints/explicit.hh>
// #include <hpp/constraints/implicit.hh>

namespace hpp {
  namespace core {
    using constraints::solver::BySubstitution;
    typedef constraints::solver::lineSearch::Backtracking Backtracking_t;
    typedef constraints::solver::lineSearch::ErrorNormBased ErrorNormBased_t;
    typedef constraints::solver::lineSearch::FixedSequence FixedSequence_t;
    typedef constraints::solver::lineSearch::Constant Constant_t;

    ConfigProjector::LineSearchType ConfigProjector::defaultLineSearch_ = ConfigProjector::FixedSequence;

    void ConfigProjector::defaultLineSearch (LineSearchType ls)
    {
      defaultLineSearch_ = ls;
    }

    HPP_DEFINE_REASON_FAILURE (REASON_MAX_ITER, "Max Iterations reached");
    HPP_DEFINE_REASON_FAILURE (REASON_ERROR_INCREASED, "Error increased");
    HPP_DEFINE_REASON_FAILURE (REASON_INFEASIBLE, "Problem infeasible");

    ConfigProjectorPtr_t ConfigProjector::create (const DevicePtr_t& robot,
						  const std::string& name,
						  value_type errorThreshold,
						  size_type maxIterations)
    {
      ConfigProjector* ptr = new ConfigProjector (robot, name, errorThreshold,
						  maxIterations);
      ConfigProjectorPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    ConfigProjectorPtr_t
    ConfigProjector::createCopy (const ConfigProjectorPtr_t cp)
    {
      ConfigProjector* ptr = new ConfigProjector (*cp);
      ConfigProjectorPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    ConfigProjector::ConfigProjector (const DevicePtr_t& robot,
				      const std::string& name,
				      value_type _errorThreshold,
				      size_type _maxIterations) :
      Constraint (name), robot_ (robot),
      lineSearchType_ (defaultLineSearch_),
      solver_ (new BySubstitution (robot->configSpace()->vectorSpacesMerged())),
      weak_ (),
      statistics_ ("ConfigProjector " + name)
    {
      errorThreshold (_errorThreshold);
      maxIterations  (_maxIterations);
      lastIsOptional (false);
      solver_->saturation(boost::make_shared<constraints::solver::saturation::Device>(robot_));
    }

    ConfigProjector::ConfigProjector (const ConfigProjector& cp) :
      Constraint (cp), robot_ (cp.robot_),
      lineSearchType_ (cp.lineSearchType_),
      solver_ (new BySubstitution(*cp.solver_)),
      weak_ (),
      statistics_ (cp.statistics_)
    {
    }

    ConfigProjector::~ConfigProjector ()
    {
      delete solver_;
    }

    ConstraintPtr_t ConfigProjector::copy () const
    {
      return createCopy (weak_.lock ());
    }

    bool ConfigProjector::contains
    (const constraints::ImplicitPtr_t& numericalConstraint) const
    {
      return solver_->contains (numericalConstraint);
    }

    bool ConfigProjector::add (const constraints::ImplicitPtr_t& nm,
			       const segments_t& passiveDofs,
			       const std::size_t priority)
    {
      return solver_->add (nm, passiveDofs, priority);
    }

    void ConfigProjector::computeValueAndJacobian
    (ConfigurationIn_t configuration, vectorOut_t value,
     matrixOut_t reducedJacobian)
    {
      Configuration_t q (configuration);
      // q_{out} = f (q_{in})
      solver_->explicitConstraintSet().solve(q);
      solver_->computeValue<true>(q);
      solver_->updateJacobian(q); // includes the jacobian of the explicit system
      solver_->getValue(value);
      solver_->getReducedJacobian(reducedJacobian);
    }

    /// Convert vector of non locked degrees of freedom to vector of
    /// all degrees of freedom
    void ConfigProjector::uncompressVector (vectorIn_t small,
					    vectorOut_t normal) const
    {
      solver_->explicitConstraintSet ().notOutDers ().transpose ().lview
        (normal) = small;
    }

    void ConfigProjector::compressVector (vectorIn_t normal,
					  vectorOut_t small) const
    {
      small = solver_->explicitConstraintSet ().notOutDers ().transpose ().rview
        (normal);
    }

    void ConfigProjector::compressMatrix (matrixIn_t normal,
					  matrixOut_t small, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixIn_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols =
          solver_->explicitConstraintSet().notOutDers();
        small = View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices());
      } else {
        small = solver_->explicitConstraintSet().notOutDers().rview(normal);
      }
    }

    void ConfigProjector::uncompressMatrix (matrixIn_t small,
					    matrixOut_t normal, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixOut_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols =
          solver_->explicitConstraintSet().notOutDers();
        View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices()) = small;
      } else {
        solver_->explicitConstraintSet().notOutDers().lview(normal) = small;
      }
    }

    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      // If configuration satisfies the constraint, do not modify it
      if (isSatisfied (configuration)) return true;
      if (!(robot_->computationFlag() & pinocchio::JACOBIAN))
        throw std::runtime_error("In ConfigProjector::apply: can't project a configuration if JACOBIAN computation flag is not enabled.");
      BySubstitution::Status status = (BySubstitution::Status)
        solverSolve (configuration);
      switch (status) {
        case BySubstitution::ERROR_INCREASED:
          statistics_.addFailure (REASON_ERROR_INCREASED);
          return false;
          break;
        case BySubstitution::MAX_ITERATION_REACHED:
          statistics_.addFailure (REASON_MAX_ITER);
          return false;
          break;
        case BySubstitution::INFEASIBLE:
          statistics_.addFailure (REASON_INFEASIBLE);
          return false;
          break;
        case BySubstitution::SUCCESS:
          statistics_.addSuccess();
          return true;
          break;
      }
      return false;
    }

    bool ConfigProjector::oneStep (ConfigurationOut_t configuration,
        vectorOut_t dq, const value_type&)
    {
      bool ret = solverOneStep (configuration);
      dq = solver_->lastStep ();
      return ret;
    }

    bool ConfigProjector::optimize (ConfigurationOut_t configuration,
        std::size_t maxIter)
    {
      if (!lastIsOptional()) return true;
      if (!isSatisfied (configuration)) return false;
      const size_type maxIterSave = maxIterations();
      if (maxIter != 0) maxIterations(maxIter);
      hppDout (info, "before optimization: " << configuration.transpose ());
      BySubstitution::Status status = BySubstitution::MAX_ITERATION_REACHED;
      switch (lineSearchType_) {
        case Backtracking  : {
                               Backtracking_t ls;
                               status = solver_->solve(configuration, true, ls);
                               break;
                             }
        case ErrorNormBased: {
                               ErrorNormBased_t ls;
                               status = solver_->solve(configuration, true, ls);
                               break;
                             }
        case FixedSequence : {
                               FixedSequence_t ls;
                               status = solver_->solve(configuration, true, ls);
                               break;
                             }
        case Constant      : {
                               Constant_t ls;
                               status = solver_->solve(configuration, true, ls);
                               break;
                             }
      }
      maxIterations(maxIterSave);
      hppDout (info, "After optimization: " << configuration.transpose ());
      if (status == BySubstitution::SUCCESS)
        return true;
      else if (status == BySubstitution::INFEASIBLE && isSatisfied (configuration))
        return true;
      else {
        hppDout (info, "Optimization failed.");
        return false;
      }
    }

    void ConfigProjector::projectVectorOnKernel (ConfigurationIn_t from,
						 vectorIn_t velocity,
						 vectorOut_t result)
    {
      solver_->projectVectorOnKernel(from, velocity, result);
    }

    void ConfigProjector::projectOnKernel (ConfigurationIn_t from,
					   ConfigurationIn_t to,
					   ConfigurationOut_t result)
    {
      solver_->projectOnKernel (from, to, result);
    }

    void ConfigProjector::addToConstraintSet
    (const ConstraintSetPtr_t& constraintSet)
    {
      if (constraintSet->configProjector ()) {
       std::ostringstream oss
         ("Constraint set cannot store more than one config-projector");
       oss << std::endl << *constraintSet;
       throw std::runtime_error (oss.str ());
      }
      // The constraint is added at the end of the set.
      Constraint::addToConstraintSet (constraintSet);
      constraintSet->removeFirstElement ();
      constraintSet->configProjectorIt_ = constraintSet->constraints_.end () - 1;
      constraintSet->trivialOrNotConfigProjectorIt_ =
	constraintSet->configProjectorIt_;
    }

    std::ostream& ConfigProjector::print (std::ostream& os) const
    {
      return os << "Config projector: " << name () << ", contains "
        << *solver_ << decindent;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config)
    {
      return solver_->isSatisfied (config);
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config,
				       vector_t& error)
    {
      error.resize (solver_->dimension() +
                    solver_->explicitConstraintSet().outDers().nbIndices());
      return solver_->isSatisfied (config, error);
    }

    const NumericalConstraints_t& ConfigProjector::numericalConstraints () const
    {
      return solver_->numericalConstraints ();
    }

    vector_t ConfigProjector::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      return solver_->rightHandSideFromConfig (config);
    }

    void ConfigProjector::rightHandSideFromConfig (
        const constraints::ImplicitPtr_t& nm,
        ConfigurationIn_t config)
    {
      if (!solver_->rightHandSideFromConfig (nm, config)) {
        throw std::runtime_error ("Function was not found in the solver.");
      }
    }

    void ConfigProjector::rightHandSide (const vector_t& small)
    {
      solver_->rightHandSide (small);
    }

    void ConfigProjector::rightHandSide (
        const constraints::ImplicitPtr_t& nm,
        vectorIn_t rhs)
    {
      if (!solver_->rightHandSide (nm, rhs)) {
        throw std::runtime_error ("Function was not found in the solver. This is probably because it is an explicit function and rhs is not supported for this type of function.");
      }
    }

    vector_t ConfigProjector::rightHandSide () const
    {
      return solver_->rightHandSide();
    }

    void ConfigProjector::rightHandSideAt (const value_type& s)
    {
      solver_->rightHandSideAt (s);
    }

    inline bool ConfigProjector::solverOneStep (ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               Backtracking_t ls;
                               return solver_->oneStep(config, ls);
                             }
        case ErrorNormBased: {
                               ErrorNormBased_t ls;
                               return solver_->oneStep(config, ls);
                             }
        case FixedSequence : {
                               FixedSequence_t ls;
                               return solver_->oneStep(config, ls);
                             }
        case Constant : {
                          Constant_t ls;
                          return solver_->oneStep(config, ls);
                        }
      }
      return false;
    }

    inline int ConfigProjector::solverSolve (
        ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               Backtracking_t ls;
                               return solver_->solve(config, ls);
                             }
        case ErrorNormBased: {
                               ErrorNormBased_t ls;
                               return solver_->solve(config, ls);
                             }
        case FixedSequence : {
                               FixedSequence_t ls;
                               return solver_->solve(config, ls);
                             }
        case Constant : {
                          Constant_t ls;
                          return solver_->solve(config, ls);
                        }
      }
      throw std::runtime_error ("Unknow line search type");
      return BySubstitution::MAX_ITERATION_REACHED;
    }

    void ConfigProjector::lastIsOptional (bool optional)
    {
      solver_->lastIsOptional(optional);
    }

    bool ConfigProjector::lastIsOptional () const
    {
      return solver_->lastIsOptional();
    }

    size_type ConfigProjector::numberFreeVariables () const
    {
      return solver_->numberFreeVariables ();
    }

    size_type ConfigProjector::numberNonLockedDof () const
    {
      return numberFreeVariables ();
    }

    size_type ConfigProjector::dimension () const
    {
      return solver_->reducedDimension();
    }

    void ConfigProjector::maxIterations (size_type iterations)
    {
      solver_->maxIterations(iterations);
    }

    size_type ConfigProjector::maxIterations () const
    {
      return solver_->maxIterations();
    }

    void ConfigProjector::errorThreshold (const value_type& threshold)
    {
      solver_->errorThreshold(threshold);
      solver_->inequalityThreshold(threshold);
    }

    value_type ConfigProjector::errorThreshold () const
    {
      return solver_->errorThreshold();
    }

    value_type ConfigProjector::residualError() const
    {
      return solver_->residualError();
    }

    const value_type& ConfigProjector::sigma() const
    {
      return solver_->sigma();
    }
  } // namespace core
} // namespace hpp
