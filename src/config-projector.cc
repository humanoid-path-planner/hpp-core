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

#include <pinocchio/multibody/liegroup/liegroup.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/constraint-set.hh>
// #include <hpp/constraints/explicit.hh>
// #include <hpp/constraints/implicit.hh>

namespace hpp {
  namespace core {
    using constraints::solver::BySubstitution;

    namespace {

      bool saturate (const DevicePtr_t& robot, vectorIn_t q, vectorOut_t qSat,
                     Eigen::VectorXi& sat)
      {
        bool ret = false;
        const se3::Model& model = robot->model();

        for (std::size_t i = 1; i < model.joints.size(); ++i) {
          const size_type nq = model.joints[i].nq();
          const size_type nv = model.joints[i].nv();
          const size_type idx_q = model.joints[i].idx_q();
          const size_type idx_v = model.joints[i].idx_v();
          for (size_type j = 0; j < nq; ++j) {
            const size_type iq = idx_q + j;
            const size_type iv = idx_v + std::min(j,nv-1);
            if        (q[iq] >= model.upperPositionLimit[iq]) {
              qSat [iq] = model.upperPositionLimit[iq];
              sat[iv] =  1;
              ret = true;
            } else if (q[iq] <= model.lowerPositionLimit[iq]) {
              qSat [iq] = model.lowerPositionLimit[iq];
              sat[iv] = -1;
              ret = true;
            } else {
              qSat [iq] = q [iq];
              sat[iv] =  0;
            }
          }
        }

        const hpp::pinocchio::ExtraConfigSpace& ecs = robot->extraConfigSpace();
        const size_type& d = ecs.dimension();

        for (size_type k = 0; k < d; ++k) {
          const size_type iq = model.nq + k;
          const size_type iv = model.nv + k;
          if        (q[iq] >= ecs.upper(k)) {
            sat[iv] =  1;
            qSat [iq] = ecs.upper(k);
            ret = true;
          } else if (q[iq] <= ecs.lower(k)) {
            qSat [iq] = ecs.lower(k);
            sat[iv] = -1;
            ret = true;
          } else {
            qSat [iq] = q [iq];
            sat[iv] =  0;
          }
        }
        return ret;
      }
    }

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
      solver_->saturation(boost::bind(saturate, robot_, _1, _2, _3));
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
      lastIsOptional(false);
      BySubstitution::Status status = (BySubstitution::Status)
        solverSolve (configuration);
      lastIsOptional(true);
      maxIterations(maxIterSave);
      hppDout (info, "After optimization: " << configuration.transpose ());
      if (status == BySubstitution::SUCCESS)
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

    inline bool ConfigProjector::solverOneStep (ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               constraints::solver::lineSearch::Backtracking ls;
                               return solver_->oneStep(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::solver::lineSearch::ErrorNormBased ls;
                               return solver_->oneStep(config, ls);
                             }
        case FixedSequence : {
                               constraints::solver::lineSearch::FixedSequence ls;
                               return solver_->oneStep(config, ls);
                             }
        case Constant : {
                          constraints::solver::lineSearch::Constant ls;
                          return solver_->oneStep(config, ls);
                        }
      }
      return false;
    }

    inline int ConfigProjector::solverSolve (
        ConfigurationOut_t config) const
    {
      typedef constraints::solver::lineSearch::Backtracking Backtracking_t;
      typedef constraints::solver::lineSearch::ErrorNormBased ErrorNormBased_t;
      typedef constraints::solver::lineSearch::FixedSequence FixedSequence_t;
      typedef constraints::solver::lineSearch::Constant Constant_t;

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
