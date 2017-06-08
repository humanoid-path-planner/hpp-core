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

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>
#include <hpp/constraints/macros.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/active-set-differentiable-function.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/comparison-type.hh>
#include <hpp/core/numerical-constraint.hh>

// #define SVD_THRESHOLD Eigen::NumTraits<value_type>::dummy_precision()
#define SVD_THRESHOLD 1e-8

namespace hpp {
  namespace core {
    using constraints::HybridSolver;

    namespace {
      HPP_DEFINE_TIMECOUNTER (projection);
      HPP_DEFINE_TIMECOUNTER (optimize);

      DifferentiableFunctionPtr_t activeSetFunction (
          const DifferentiableFunctionPtr_t& function,
          const SizeIntervals_t& pdofs)
      {
        if (pdofs.empty()) return function;
        return constraints::ActiveSetDifferentiableFunctionPtr_t (new constraints::ActiveSetDifferentiableFunction(function, pdofs));
      }

      template <typename T> bool convert (const ComparisonTypePtr_t& c,
          HybridSolver::ComparisonTypes_t& types,
          HybridSolver::ComparisonType t)
      {
        if (HPP_DYNAMIC_PTR_CAST(T, c)) { types.push_back (t); return true; }
        return false;
      }

      void convertCompTypes (
          const ComparisonTypePtr_t& c,
          HybridSolver::ComparisonTypes_t& types)
      {
        ComparisonTypesPtr_t cts = HPP_DYNAMIC_PTR_CAST(ComparisonTypes, c);
        if (cts) {
          for (std::size_t i = 0; i < cts->size(); ++i)
            convertCompTypes(cts->at(i), types);
        } else if (convert<SuperiorIneq>(c, types, HybridSolver::Superior   )) {}
        else if   (convert<InferiorIneq>(c, types, HybridSolver::Inferior   )) {}
        else if   (convert<EqualToZero >(c, types, HybridSolver::EqualToZero)) {}
        else if   (convert<Equality    >(c, types, HybridSolver::Equality   )) {}
        else throw std::logic_error("Unknow ComparisonType");
      }
    }

    HPP_DEFINE_REASON_FAILURE (REASON_MAX_ITER, "Max Iterations reached");
    HPP_DEFINE_REASON_FAILURE (REASON_ERROR_INCREASED, "Error increased");

    //using boost::fusion::result_of::at;
    bool operator< (const LockedJointPtr_t& l1, const LockedJointPtr_t& l2)
    {
      return l1->rankInVelocity () < l2->rankInVelocity ();
    }

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
				      value_type errorThreshold,
				      size_type maxIterations) :
      Constraint (name), robot_ (robot), functions_ (),
      passiveDofs_ (), lockedJoints_ (),
      rhsReducedSize_ (0),
      toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      nbNonLockedDofs_ (robot_->numberDof ()),
      nbLockedDofs_ (0),
      explicitComputation_ (false),
      lineSearchType_ (Default),
      // lineSearchType_ (Backtracking),
      solver_ (robot->configSize(), robot->numberDof()),
      weak_ (),
      statistics_ ("ConfigProjector " + name)
    {
      solver_.errorThreshold(errorThreshold);
      solver_.inequalityThreshold(errorThreshold);
      solver_.maxIterations(maxIterations);
      solver_.lastIsOptional(false);
      solver_.integration(boost::bind(hpp::pinocchio::integrate<true, se3::LieGroupTpl>, robot_, _1, _2, _3));
    }

    ConfigProjector::ConfigProjector (const ConfigProjector& cp) :
      Constraint (cp), robot_ (cp.robot_),
      functions_ (cp.functions_),
      passiveDofs_ (cp.passiveDofs_), lockedJoints_ (),
      intervals_ (cp.intervals_),
      rightHandSide_ (cp.rightHandSide_),
      rhsReducedSize_ (cp.rhsReducedSize_),
      toMinusFrom_ (cp.toMinusFrom_.size ()),
      projMinusFrom_ (cp.projMinusFrom_.size ()),
      nbNonLockedDofs_ (cp.nbNonLockedDofs_), nbLockedDofs_ (cp.nbLockedDofs_),
      explicitComputation_ (cp.explicitComputation_),
      lineSearchType_ (cp.lineSearchType_),
      solver_ (cp.solver_),
      weak_ (),
      statistics_ (cp.statistics_)
    {
      for (LockedJoints_t::const_iterator it = cp.lockedJoints_.begin ();
	   it != cp.lockedJoints_.end (); ++it) {
	lockedJoints_.push_back (HPP_STATIC_PTR_CAST (LockedJoint,
						      (*it)->copy ()));
      }
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
			       const SizeIntervals_t& passiveDofs,
			       const std::size_t priority)
    {
      if (contains (nm)) {
	hppDout (error, "Constraint " << nm->functionPtr()->name ()
		 << " already in " << this->name () << "." << std::endl);
	return false;
      }
      bool addedAsExplicit = false;
      ExplicitNumericalConstraintPtr_t enm =
        HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nm);
      if (enm) {
        addedAsExplicit = solver_.explicitSolver().add(enm->explicitFunction(),
            Eigen::RowBlockIndexes(enm->inputConf()),
            Eigen::RowBlockIndexes(enm->outputConf()),
            Eigen::ColBlockIndexes(enm->inputVelocity()),
            Eigen::RowBlockIndexes(enm->outputVelocity()));
      }

      if (!addedAsExplicit) {
        HybridSolver::ComparisonTypes_t types;
        convertCompTypes(nm->comparisonType(), types);
        solver_.add(activeSetFunction(nm->functionPtr(), passiveDofs), priority, types);
      } else {
        hppDout (info, "Numerical constraint added as explicit function: " <<
            enm->explicitFunction()->name());
        solver_.explicitSolverHasChanged();
      }

      functions_.push_back (nm);
      passiveDofs_.push_back (passiveDofs);
      rhsReducedSize_ += nm->rhsSize ();
      // TODO: no need to recompute intervals.
      computeIntervals ();
      resize ();
      updateExplicitComputation ();
      return true;
    }

    void ConfigProjector::computeIntervals ()
    {
      intervals_.clear ();
      nbLockedDofs_ = 0;
      std::pair < size_type, size_type > interval;
      std::size_t latestIndex = 0;
      size_type size;
      lockedJoints_.sort ();
      // temporarily add an element at the end of the list.
      lockedJoints_.push_back (LockedJoint::create (robot_));
      for (LockedJoints_t::const_iterator itLocked = lockedJoints_.begin ();
	   itLocked != lockedJoints_.end (); ++itLocked) {
    std::size_t index = (*itLocked)->rankInVelocity ();
	nbLockedDofs_ += (*itLocked)->numberDof ();
	hppDout (info, "number locked dof " << (*itLocked)->numberDof ());
	size = (index - latestIndex);
	if (size > 0) {
	  interval.first = latestIndex;
	  interval.second = size;
	  intervals_.push_back (interval);
	}
    latestIndex = index + (*itLocked)->numberDof ();
      }
      // Remove temporary element.
      lockedJoints_.pop_back ();
    }

    void ConfigProjector::resize ()
    {
      std::size_t sizeOutput = solver_.dimension();
      nbNonLockedDofs_ = robot_->numberDof () - nbLockedDofs_;
      rightHandSide_ = vector_t::Zero (sizeOutput);
      projMinusFrom_.setZero ();
    }

    void ConfigProjector::computeValueAndJacobian
    (ConfigurationIn_t configuration, vectorOut_t value,
     matrixOut_t reducedJacobian)
    {
      solver_.computeValue<true>(configuration);
      solver_.updateJacobian(configuration); // includes the jacobian of the explicit system
      solver_.getValue(value);
      solver_.getReducedJacobian(reducedJacobian);
    }

    /// Convert vector of non locked degrees of freedom to vector of
    /// all degrees of freedom
    void ConfigProjector::uncompressVector (vectorIn_t small,
					    vectorOut_t normal) const
    {
      assert (small.size () + nbLockedDofs_ == robot_->numberDof ());
      assert (normal.size () == robot_->numberDof ());
      if (intervals_.empty ()) {
	normal = small;
	return;
      }
      size_type col = 0;
      for (SizeIntervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); ++itInterval) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	normal.segment (col0, nbCols) = small.segment (col, nbCols);
	col += itInterval->second;
      }
    }

    void ConfigProjector::compressVector (vectorIn_t normal,
					  vectorOut_t small) const
    {
      assert (small.size () + nbLockedDofs_ == robot_->numberDof ());
      assert (normal.size () == robot_->numberDof ());
      if (intervals_.empty ()) {
	small = normal;
	return;
      }
      size_type col = 0;
      for (SizeIntervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); ++itInterval) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	small.segment (col, nbCols) = normal.segment (col0, nbCols);
	col += itInterval->second;
      }
    }

    void ConfigProjector::compressMatrix (matrixIn_t normal,
					  matrixOut_t small, bool rows) const
    {
      if (intervals_.empty ()) {
	small = normal;
	return;
      }
      size_type col = 0;
      for (SizeIntervals_t::const_iterator itCol = intervals_.begin ();
	   itCol != intervals_.end (); ++itCol) {
	size_type col0 = itCol->first;
	size_type nbCols = itCol->second;
	if (rows) {
	  size_type row = 0;
	  for (SizeIntervals_t::const_iterator itRow = intervals_.begin ();
	       itRow != intervals_.end (); ++itRow) {
	    size_type row0 = itRow->first;
	    size_type nbRows = itRow->second;
	    small.block (row, col, nbRows, nbCols) =
	      normal.block (row0, col0, nbRows, nbCols);
	    row += nbRows;
	  }
	  assert (row == small.rows ());
	} else {
	  small.middleCols (col, nbCols) = normal.middleCols (col0, nbCols);
	}
	col += nbCols;
      }
      assert (col == small.cols ());
    }

    void ConfigProjector::uncompressMatrix (matrixIn_t small,
					    matrixOut_t normal, bool rows) const
    {
      if (intervals_.empty ()) {
	normal = small;
	return;
      }
      size_type col = 0;
      for (SizeIntervals_t::const_iterator itCol = intervals_.begin ();
	   itCol != intervals_.end (); ++itCol) {
	size_type col0 = itCol->first;
	size_type nbCols = itCol->second;
	if (rows) {
	  size_type row = 0;
	  for (SizeIntervals_t::const_iterator itRow = intervals_.begin ();
	       itRow != intervals_.end (); ++itRow) {
	    size_type row0 = itRow->first;
	    size_type nbRows = itRow->second;
	    normal.block (row0, col0, nbRows, nbCols) =
	      small.block (row, col, nbRows, nbCols);
	    row += nbRows;
	  }
	  assert (row == small.rows ());
	} else {
	  normal.middleCols (col0, nbCols) = small.middleCols (col, nbCols);
	}
	col += nbCols;
      }
      assert (col == small.cols ());
    }

    void ConfigProjector::updateExplicitComputation ()
    {
      if (functions_.empty()) {
        explicitComputation_ = true;
        return;
      }
      if ((functions_.size () != 1) || (explicitFunctions_.size () != 1)) {
	explicitComputation_ = false;
	return;
      }
      HPP_STATIC_CAST_REF_CHECK (ExplicitNumericalConstraint,
				 *(functions_ [0]));
      const SizeIntervals_t& output =
	HPP_STATIC_PTR_CAST (ExplicitNumericalConstraint,
			     functions_ [0])->outputConf ();
      for (LockedJoints_t::const_iterator itLocked = lockedJoints_.begin ();
	   itLocked != lockedJoints_.end (); ++itLocked) {
	size_type a = (size_type) (*itLocked)->rankInConfiguration ();
	size_type b = (size_type) (*itLocked)->rankInConfiguration () +
	  (*itLocked)->size () - 1;
	for (SizeIntervals_t::const_iterator itOut = output.begin ();
	     itOut != output.end (); ++itOut) {
	  size_type c = itOut->first;
	  size_type d = itOut->first + itOut->second - 1;
	  if ((d >= a) && (b >= c)) {
	    explicitComputation_ = false;
	    return;
	  }
	}
      }
      explicitComputation_ = true;
    }

    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      hppDout (info, "before projection: " << configuration.transpose ());
      assert (!configuration.hasNaN());
      /*
      if (explicitComputation_) {
	hppDout (info, "Explicit computation: " <<
		 functions_ [0]->functionPtr ()->name ());
	HPP_STATIC_CAST_REF_CHECK (ExplicitNumericalConstraint,
				   *(functions_ [0]));
	HPP_STATIC_PTR_CAST (ExplicitNumericalConstraint,
			     functions_ [0])->solve (configuration);
      }
      */
      HPP_START_TIMECOUNTER (projection);
      HybridSolver::Status status = solverSolve (configuration);
      HPP_STOP_TIMECOUNTER (projection);
      HPP_DISPLAY_TIMECOUNTER (projection);
      assert (!configuration.hasNaN());
      switch (status) {
        case HybridSolver::ERROR_INCREASED:
          statistics_.addFailure (REASON_ERROR_INCREASED);
          statistics_.isLowRatio (true);
          return false;
          break;
        case HybridSolver::MAX_ITERATION_REACHED:
          statistics_.addFailure (REASON_MAX_ITER);
          statistics_.isLowRatio (true);
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
        vectorOut_t dq, const value_type& alpha)
    {
      // TODO dq = solver_.dq_; // Not accessible yet.
      return solverOneStep (configuration);
    }

    bool ConfigProjector::optimize (ConfigurationOut_t configuration,
        std::size_t maxIter, const value_type alpha)
    {
      if (!lastIsOptional()) return true;
      if (!isSatisfied (configuration)) return false;
      const size_type maxIterSave = maxIterations();
      if (maxIter == 0) maxIterations(maxIter);
      hppDout (info, "before optimization: " << configuration.transpose ());
      HPP_START_TIMECOUNTER (optimize);
      lastIsOptional(false);
      HybridSolver::Status status = solverSolve (configuration);
      lastIsOptional(true);
      maxIterations(maxIterSave);
      HPP_STOP_TIMECOUNTER (optimize);
      HPP_DISPLAY_TIMECOUNTER (optimize);
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
      solver_.projectOnKernel(from, velocity, result);
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
	  *itLock = lockedJoint;

          solver_.explicitSolver().replace((*itLock)->function(), lockedJoint->function());

	  return;
	}
      }

      solver_.explicitSolver().add(lockedJoint->function(),
          Eigen::RowBlockIndexes(),
          Eigen::RowBlockIndexes(SizeInterval_t(lockedJoint->rankInConfiguration(), lockedJoint->size())),
          Eigen::ColBlockIndexes(),
          Eigen::RowBlockIndexes(SizeInterval_t(lockedJoint->rankInVelocity(), lockedJoint->numberDof())));
      solver_.explicitSolverHasChanged();

      lockedJoints_.push_back (lockedJoint);
      hppDout (info, "add locked joint " << lockedJoint->jointName_
	       << " rank in velocity: " << lockedJoint->rankInVelocity ()
	       << ", size: " << lockedJoint->numberDof ());
      computeIntervals ();
      hppDout (info, "Intervals: ");
      for (SizeIntervals_t::const_iterator it = intervals_.begin ();
	   it != intervals_.end (); ++it) {
	hppDout (info, "[" << it->first << "," << it->first + it->second - 1
		 << "]");
      }
      resize ();
      updateExplicitComputation ();
      if (!lockedJoint->comparisonType ()->constantRightHandSide ())
        rhsReducedSize_ += lockedJoint->rhsSize ();
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
      os << "Config projector: " << name () << ", contains" << std::endl;
      for (NumericalConstraints_t::const_iterator it = functions_.begin ();
	   it != functions_.end (); ++it) {
	const DifferentiableFunction& f = (*it)->function ();
	os << "    " << f << std::endl;
      }
      os << "    Locked dofs" << std::endl;
      for (LockedJoints_t::const_iterator itLock = lockedJoints_.begin ();
          itLock != lockedJoints_.end (); ++itLock) {
	const LockedJoint& lj (*(itLock->get ()));
	os << "      ";
	os << lj << std::endl;
      }
      os << "    Intervals: ";
      for (SizeIntervals_t::const_iterator it=intervals_.begin ();
	   it != intervals_.end (); ++it) {
	os << "[" << it->first << "," << it->first + it->second - 1 << "], ";
      }
      os << std::endl;
      return os;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config)
    {
      return solver_.isSatisfied (config);
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config,
				       vector_t& error)
    {
      error.resize (solver_.dimension() + solver_.explicitSolver().outDers().nbIndexes());
      return solver_.isSatisfied (config, error);
    }

    vector_t ConfigProjector::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      solver_.rightHandSideFromInput (config);

      // Update other degrees of freedom.
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it )
        (*it)->rightHandSideFromConfig (config);
      return rightHandSide();
    }

    void ConfigProjector::rightHandSideFromConfig (
        const NumericalConstraintPtr_t& nm,
        ConfigurationIn_t config)
    {
      solver_.rightHandSideFromInput (nm->functionPtr(), config);
    }

    void ConfigProjector::rightHandSideFromConfig (
        const LockedJointPtr_t& lj,
        ConfigurationIn_t config)
    {
      lj->rightHandSideFromConfig (config);
    }

    void ConfigProjector::rightHandSide (const vector_t& small)
    {
      const size_type rhsImplicitSize = solver_.rightHandSideSize();
      solver_.rightHandSide (small.head(rhsImplicitSize));

      assert (rightHandSide_.size () == rhsImplicitSize); // TODO remove
      size_type row = rhsImplicitSize;
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it ) {
        LockedJoint& lj = **it;
        if (!lj.comparisonType ()->constantRightHandSide ()) {
          lj.rightHandSide (small.segment (row, lj.rhsSize ()));
          row += lj.rhsSize ();
        }
      }
      assert (row == small.size ());
    }

    vector_t ConfigProjector::rightHandSide () const
    {
      vector_t small(rhsReducedSize_);

      vector_t rhsImplicit = solver_.rightHandSide();

      size_type row = rhsImplicit.size();
      small.head(row) = rhsImplicit;

      for (LockedJoints_t::const_iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it ) {
        LockedJoint& lj = **it;
        if (!lj.comparisonType ()->constantRightHandSide ()) {
          small.segment (row, lj.rhsSize ()) = lj.rightHandSide ();
          row += lj.rhsSize ();
        }
      }
      assert (row == small.size ());
      return small;
    }

    inline bool ConfigProjector::solverOneStep (ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               constraints::lineSearch::Backtracking ls;
                               return solver_.oneStep(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return solver_.oneStep(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return solver_.oneStep(config, ls);
                             }
      }
      return false;
    }

    inline HybridSolver::Status ConfigProjector::solverSolve (
        ConfigurationOut_t config) const
    {
      switch (lineSearchType_) {
        case Backtracking  : {
                               constraints::lineSearch::Backtracking ls;
                               return solver_.solve(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return solver_.solve(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return solver_.solve(config, ls);
                             }
      }
      throw std::runtime_error ("Unknow line search type");
      return HybridSolver::MAX_ITERATION_REACHED;
    }
  } // namespace core
} // namespace hpp
