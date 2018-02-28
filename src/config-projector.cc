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
#include <hpp/constraints/active-set-differentiable-function.hh>
#include <hpp/constraints/hybrid-solver.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/numerical-constraint.hh>

namespace hpp {
  namespace core {
    using constraints::HybridSolver;

    namespace {

      DifferentiableFunctionPtr_t activeSetFunction (
          const DifferentiableFunctionPtr_t& function,
          const segments_t& pdofs)
      {
        if (pdofs.empty()) return function;
        return constraints::ActiveSetDifferentiableFunctionPtr_t (new constraints::ActiveSetDifferentiableFunction(function, pdofs));
      }

      bool saturate (const DevicePtr_t& robot, vectorIn_t q, Eigen::VectorXi& sat)
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
              sat[iv] =  1;
              ret = true;
            } else if (q[iq] <= model.lowerPositionLimit[iq]) {
              sat[iv] = -1;
              ret = true;
            } else
              sat[iv] =  0;
          }
        }

        const hpp::pinocchio::ExtraConfigSpace& ecs = robot->extraConfigSpace();
        const size_type& d = ecs.dimension();

        for (size_type k = 0; k < d; ++k) {
          const size_type iq = model.nq + k;
          const size_type iv = model.nv + k;
          if        (q[iq] >= ecs.upper(k)) {
            sat[iv] =  1;
            ret = true;
          } else if (q[iq] <= ecs.lower(k)) {
            sat[iv] = -1;
            ret = true;
          } else
            sat[iv] =  0;
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
      Constraint (name), robot_ (robot), functions_ (),
      lockedJoints_ (),
      toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      lineSearchType_ (defaultLineSearch_),
      solver_ (new HybridSolver (robot->configSize(), robot->numberDof())),
      weak_ (),
      statistics_ ("ConfigProjector " + name)
    {
      errorThreshold (_errorThreshold);
      maxIterations  (_maxIterations);
      lastIsOptional (false);
      solver_->integration(boost::bind(hpp::pinocchio::integrate<true, se3::LieGroupTpl>, robot_, _1, _2, _3));
      solver_->saturation(boost::bind(saturate, robot_, _1, _2));
    }

    ConfigProjector::ConfigProjector (const ConfigProjector& cp) :
      Constraint (cp), robot_ (cp.robot_),
      functions_ (cp.functions_),
      lockedJoints_ (),
      toMinusFrom_ (cp.toMinusFrom_.size ()),
      projMinusFrom_ (cp.projMinusFrom_.size ()),
      lineSearchType_ (cp.lineSearchType_),
      solver_ (new HybridSolver(*cp.solver_)),
      weak_ (),
      statistics_ (cp.statistics_)
    {
      // TODO remove me
      for (LockedJoints_t::const_iterator it = cp.lockedJoints_.begin ();
	   it != cp.lockedJoints_.end (); ++it) {
        LockedJointPtr_t lj = HPP_STATIC_PTR_CAST (LockedJoint, (*it)->copy ());
        if (!solver_->explicitSolver().replace((*it)->explicitFunction(), lj->explicitFunction()))
          throw std::runtime_error("Could not replace lockedJoint function");
	lockedJoints_.push_back (lj);
      }
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
    (const NumericalConstraintPtr_t& numericalConstraint) const
    {
      for (NumericalConstraints_t::const_iterator it = functions_.begin ();
	   it != functions_.end (); ++it) {
	if (numericalConstraint == *it || *numericalConstraint == **it)
	  return true;
      }
      return false;
    }

    bool ConfigProjector::add (const NumericalConstraintPtr_t& nm,
			       const segments_t& passiveDofs,
			       const std::size_t priority)
    {
      if (contains (nm)) {
	hppDout (error, "Constraint " << nm->functionPtr()->name ()
		 << " already in " << this->name () << "." << std::endl);
	return false;
      }
      constraints::ComparisonTypes_t types = nm->comparisonType();

      LockedJointPtr_t lj = HPP_DYNAMIC_PTR_CAST (LockedJoint, nm);
      assert (!lj);

      bool addedAsExplicit = false;
      ExplicitNumericalConstraintPtr_t enm =
        HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nm);
      if (enm) {
        addedAsExplicit = solver_->explicitSolver().add(enm->explicitFunction(),
            Eigen::RowBlockIndices(enm->inputConf()),
            Eigen::RowBlockIndices(enm->outputConf()),
            Eigen::ColBlockIndices(enm->inputVelocity()),
            Eigen::RowBlockIndices(enm->outputVelocity()),
            types) >= 0;
        if (addedAsExplicit && enm->outputFunction() && enm->outputFunctionInverse()) {
          bool ok = solver_->explicitSolver().setG (enm->explicitFunction(),
              enm->outputFunction(), enm->outputFunctionInverse());
          assert (ok);
        }
        if (!addedAsExplicit) {
          hppDout (info, "Could not treat " <<
              enm->explicitFunction()->name() << " as an explicit function."
              );
        }
      }

      if (!addedAsExplicit) {
        solver_->add(activeSetFunction(nm->functionPtr(), passiveDofs), priority, types);
      } else {
        hppDout (info, "Numerical constraint added as explicit function: "
            << enm->explicitFunction()->name() << "with "
            << "input conf " << Eigen::RowBlockIndices(enm->inputConf())
            << "input vel" << Eigen::RowBlockIndices(enm->inputVelocity())
            << "output conf " << Eigen::RowBlockIndices(enm->outputConf())
            << "output vel " << Eigen::RowBlockIndices(enm->outputVelocity()));
        solver_->explicitSolverHasChanged();
      }
      hppDout (info, "Constraints " << name() << " has dimension " << solver_->dimension());

      functions_.push_back (nm);
      return true;
    }

    void ConfigProjector::computeValueAndJacobian
    (ConfigurationIn_t configuration, vectorOut_t value,
     matrixOut_t reducedJacobian)
    {
      Configuration_t q (configuration);
      solver_->explicitSolver().solve(q);
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
      solver_->explicitSolver().freeDers().transpose().lview(normal) = small;
    }

    void ConfigProjector::compressVector (vectorIn_t normal,
					  vectorOut_t small) const
    {
      small = solver_->explicitSolver().freeDers().transpose().rview(normal);
    }

    void ConfigProjector::compressMatrix (matrixIn_t normal,
					  matrixOut_t small, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixIn_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols = solver_->explicitSolver().freeDers();
        small = View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices());
      } else {
        small = solver_->explicitSolver().freeDers().rview(normal);
      }
    }

    void ConfigProjector::uncompressMatrix (matrixIn_t small,
					    matrixOut_t normal, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixOut_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols = solver_->explicitSolver().freeDers();
        View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices()) = small;
      } else {
        solver_->explicitSolver().freeDers().lview(normal) = small;
      }
    }

    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      HybridSolver::Status status = (HybridSolver::Status)solverSolve (configuration);
      switch (status) {
        case HybridSolver::ERROR_INCREASED:
          statistics_.addFailure (REASON_ERROR_INCREASED);
          return false;
          break;
        case HybridSolver::MAX_ITERATION_REACHED:
          statistics_.addFailure (REASON_MAX_ITER);
          return false;
          break;
        case HybridSolver::INFEASIBLE:
          statistics_.addFailure (REASON_INFEASIBLE);
          return false;
          break;
        case HybridSolver::SUCCESS:
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
      HybridSolver::Status status = (HybridSolver::Status)solverSolve (configuration);
      lastIsOptional(true);
      maxIterations(maxIterSave);
      hppDout (info, "After optimization: " << configuration.transpose ());
      if (status == HybridSolver::SUCCESS)
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
      // TODO equivalent
      if (functions_.empty ()) {
        result = velocity;
        return;
      }
      solver_->projectOnKernel(from, velocity, result);
    }

    void ConfigProjector::projectOnKernel (ConfigurationIn_t from,
					   ConfigurationIn_t to,
					   ConfigurationOut_t result)
    {
      // TODO equivalent
      if (functions_.empty ()) {
        result = to;
        return;
      }
      pinocchio::difference<se3::LieGroupTpl> (robot_, to, from, toMinusFrom_);
      projectVectorOnKernel (from, toMinusFrom_, projMinusFrom_);
      pinocchio::integrate<true, se3::LieGroupTpl> (robot_, from, projMinusFrom_, result);
    }

    void ConfigProjector::add (const LockedJointPtr_t& lockedJoint)
    {
      if (lockedJoint->numberDof () == 0) return;
      // If the same dof is already locked, replace by new value
      for (LockedJoints_t::iterator itLock = lockedJoints_.begin ();
	   itLock != lockedJoints_.end (); ++itLock) {
	if (lockedJoint->rankInVelocity () == (*itLock)->rankInVelocity ()) {
          if (!solver_->explicitSolver().replace((*itLock)->explicitFunction(), lockedJoint->explicitFunction())) {
            throw std::runtime_error("Could not replace lockedJoint function " + lockedJoint->jointName_);
          }
	  *itLock = lockedJoint;
	  return;
	}
      }

      constraints::ComparisonTypes_t types = lockedJoint->comparisonType();

      bool added = solver_->explicitSolver().add(lockedJoint->explicitFunction(),
          Eigen::RowBlockIndices(lockedJoint->inputConf()),
          Eigen::RowBlockIndices(lockedJoint->outputConf()),
          Eigen::ColBlockIndices(lockedJoint->inputVelocity()),
          Eigen::RowBlockIndices(lockedJoint->outputVelocity()),
          types) >= 0;

      if (!added) {
        throw std::runtime_error("Could not add lockedJoint function " + lockedJoint->jointName_);
      }
      if (added) {
        solver_->explicitSolver().rightHandSide (
            lockedJoint->explicitFunction(),
            lockedJoint->rightHandSide());
      }
      solver_->explicitSolverHasChanged();

      lockedJoints_.push_back (lockedJoint);
      hppDout (info, "add locked joint " << lockedJoint->jointName_
	       << " rank in velocity: " << lockedJoint->rankInVelocity ()
	       << ", size: " << lockedJoint->numberDof ());
      hppDout (info, "Intervals: " << solver_->explicitSolver().outDers());
      hppDout (info, "Constraints " << name() << " has dimension " << solver_->dimension());
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
      error.resize (solver_->dimension() + solver_->explicitSolver().outDers().nbIndices());
      return solver_->isSatisfied (config, error);
    }

    vector_t ConfigProjector::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      return solver_->rightHandSideFromInput (config);
    }

    void ConfigProjector::rightHandSideFromConfig (
        const NumericalConstraintPtr_t& nm,
        ConfigurationIn_t config)
    {
      ExplicitNumericalConstraintPtr_t enm =
        HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nm);
      DifferentiableFunctionPtr_t fImplicit = nm->functionPtr(), fExplicit;
      if (enm) fExplicit = enm->explicitFunction();

      if (!solver_->rightHandSideFromInput (
            fImplicit, fExplicit, config)) {
        throw std::runtime_error ("Function was not found in the solver.");
      }
    }

    void ConfigProjector::rightHandSideFromConfig (
        const LockedJointPtr_t& lj,
        ConfigurationIn_t config)
    {
      solver_->explicitSolver().rightHandSideFromInput (
          lj->explicitFunction(), config);
    }

    void ConfigProjector::rightHandSide (const vector_t& small)
    {
      solver_->rightHandSide (small);
    }

    void ConfigProjector::rightHandSide (
        const NumericalConstraintPtr_t& nm,
        vectorIn_t rhs)
    {
      ExplicitNumericalConstraintPtr_t enm =
        HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nm);
      DifferentiableFunctionPtr_t fImplicit = nm->functionPtr(), fExplicit;
      if (enm) fExplicit = enm->explicitFunction();

      if (!solver_->rightHandSide (fImplicit, fExplicit, rhs)) {
        throw std::runtime_error ("Function was not found in the solver. This is probably because it is an explicit function and rhs is not supported for this type of function.");
      }
    }

    void ConfigProjector::rightHandSide (
        const LockedJointPtr_t& lj,
        vectorIn_t rhs)
    {
      solver_->explicitSolver().rightHandSide (lj->explicitFunction(), rhs);
    }

    vector_t ConfigProjector::rightHandSide () const
    {
      return solver_->rightHandSide();
    }

    inline bool ConfigProjector::solverOneStep (ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               constraints::lineSearch::Backtracking ls;
                               return solver_->oneStep(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return solver_->oneStep(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return solver_->oneStep(config, ls);
                             }
        case Constant : {
                          constraints::lineSearch::Constant ls;
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
                               constraints::lineSearch::Backtracking ls;
                               return solver_->solve(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return solver_->solve(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return solver_->solve(config, ls);
                             }
        case Constant : {
                          constraints::lineSearch::Constant ls;
                          return solver_->solve(config, ls);
                        }
      }
      throw std::runtime_error ("Unknow line search type");
      return HybridSolver::MAX_ITERATION_REACHED;
    }

    void ConfigProjector::lastIsOptional (bool optional)
    {
      solver_->lastIsOptional(optional);
    }

    bool ConfigProjector::lastIsOptional () const
    {
      return solver_->lastIsOptional();
    }

    size_type ConfigProjector::numberNonLockedDof () const
    {
      return solver_->explicitSolver().freeDers().nbIndices();
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
