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
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/constraints/svd.hh>
#include <hpp/constraints/macros.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/active-set-differentiable-function.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/comparison-type.hh>
#include <hpp/core/numerical-constraint.hh>

namespace hpp {
  namespace core {
    using constraints::HybridSolver;

    namespace {

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
      rhsReducedSize_ (0),
      toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      lineSearchType_ (Default),
      // lineSearchType_ (Backtracking),
      minimalSolver_ (robot->configSize(), robot->numberDof()),
      fullSolver_ (robot->configSize(), robot->numberDof()),
      weak_ (),
      statistics_ ("ConfigProjector " + name)
    {
      errorThreshold (_errorThreshold);
      maxIterations  (_maxIterations);
      lastIsOptional (false);
      minimalSolver_.integration(boost::bind(hpp::pinocchio::integrate<true, se3::LieGroupTpl>, robot_, _1, _2, _3));
      minimalSolver_.explicitSolver().difference (boost::bind(hpp::pinocchio::difference<se3::LieGroupTpl>, robot, _1, _2, _3));
      fullSolver_.explicitSolver().difference (boost::bind(hpp::pinocchio::difference<se3::LieGroupTpl>, robot, _1, _2, _3));
    }

    ConfigProjector::ConfigProjector (const ConfigProjector& cp) :
      Constraint (cp), robot_ (cp.robot_),
      functions_ (cp.functions_),
      lockedJoints_ (),
      rightHandSide_ (cp.rightHandSide_),
      rhsReducedSize_ (cp.rhsReducedSize_),
      toMinusFrom_ (cp.toMinusFrom_.size ()),
      projMinusFrom_ (cp.projMinusFrom_.size ()),
      lineSearchType_ (cp.lineSearchType_),
      minimalSolver_ (cp.minimalSolver_),
      fullSolver_ (cp.fullSolver_),
      weak_ (),
      statistics_ (cp.statistics_)
    {
      for (LockedJoints_t::const_iterator it = cp.lockedJoints_.begin ();
	   it != cp.lockedJoints_.end (); ++it) {
        LockedJointPtr_t lj = HPP_STATIC_PTR_CAST (LockedJoint, (*it)->copy ());
        if (!minimalSolver_.explicitSolver().replace((*it)->function(), lj->function())
            || !fullSolver_.explicitSolver().replace((*it)->function(), lj->function()))
          throw std::runtime_error("Could not replace lockedJoint function");
	lockedJoints_.push_back (lj);
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
        addedAsExplicit = minimalSolver_.explicitSolver().add(enm->explicitFunction(),
            Eigen::RowBlockIndices(enm->inputConf()),
            Eigen::RowBlockIndices(enm->outputConf()),
            Eigen::ColBlockIndices(enm->inputVelocity()),
            Eigen::RowBlockIndices(enm->outputVelocity()));
        if (!addedAsExplicit) {
          hppDout (info, "Could not treat " <<
              enm->explicitFunction()->name() << " as an explicit function."
              );
        }
      }

      HybridSolver::ComparisonTypes_t types;
      convertCompTypes(nm->comparisonType(), types);
      if (!addedAsExplicit) {
        minimalSolver_.add(activeSetFunction(nm->functionPtr(), passiveDofs), priority, types);
      } else {
        hppDout (info, "Numerical constraint added as explicit function: "
            << enm->explicitFunction()->name() << "with "
            << "input conf " << Eigen::RowBlockIndices(enm->inputConf())
            << "input vel" << Eigen::RowBlockIndices(enm->inputVelocity())
            << "output conf " << Eigen::RowBlockIndices(enm->outputConf())
            << "output vel " << Eigen::RowBlockIndices(enm->outputVelocity()));
        minimalSolver_.explicitSolverHasChanged();
      }
      fullSolver_.add(activeSetFunction(nm->functionPtr(), passiveDofs), priority, types);

      functions_.push_back (nm);
      // rhsReducedSize_ += nm->rhsSize ();
      return true;
    }

    void ConfigProjector::computeValueAndJacobian
    (ConfigurationIn_t configuration, vectorOut_t value,
     matrixOut_t reducedJacobian)
    {
      minimalSolver_.computeValue<true>(configuration);
      minimalSolver_.updateJacobian(configuration); // includes the jacobian of the explicit system
      minimalSolver_.getValue(value);
      minimalSolver_.getReducedJacobian(reducedJacobian);
    }

    /// Convert vector of non locked degrees of freedom to vector of
    /// all degrees of freedom
    void ConfigProjector::uncompressVector (vectorIn_t small,
					    vectorOut_t normal) const
    {
      minimalSolver_.explicitSolver().inDers().lviewTranspose(normal) = small;
    }

    void ConfigProjector::compressVector (vectorIn_t normal,
					  vectorOut_t small) const
    {
      small = minimalSolver_.explicitSolver().inDers().rviewTranspose(normal);
    }

    void ConfigProjector::compressMatrix (matrixIn_t normal,
					  matrixOut_t small, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixIn_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols = minimalSolver_.explicitSolver().inDers();
        small = View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices());
      } else {
        small = minimalSolver_.explicitSolver().inDers().rview(normal);
      }
    }

    void ConfigProjector::uncompressMatrix (matrixIn_t small,
					    matrixOut_t normal, bool rows) const
    {
      if (rows) {
        typedef Eigen::MatrixBlockView<matrixOut_t, Eigen::Dynamic, Eigen::Dynamic, false, false> View;
        const Eigen::ColBlockIndices& cols = minimalSolver_.explicitSolver().inDers();
        View (normal, cols.nbIndices(), cols.indices(), cols.nbIndices(), cols.indices()) = small;
      } else {
        minimalSolver_.explicitSolver().inDers().lview(normal) = small;
      }
    }

    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      HybridSolver::Status status = solverSolve (configuration);
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
      // TODO dq = minimalSolver_.dq_; // Not accessible yet.
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
      lastIsOptional(false);
      HybridSolver::Status status = solverSolve (configuration);
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
      minimalSolver_.projectOnKernel(from, velocity, result);
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

          minimalSolver_.explicitSolver().replace((*itLock)->function(), lockedJoint->function());
          fullSolver_.explicitSolver().replace((*itLock)->function(), lockedJoint->function());

	  return;
	}
      }

      bool added = minimalSolver_.explicitSolver().add(lockedJoint->function(),
          Eigen::RowBlockIndices(),
          Eigen::RowBlockIndices(SizeInterval_t(lockedJoint->rankInConfiguration(), lockedJoint->size())),
          Eigen::ColBlockIndices(),
          Eigen::RowBlockIndices(SizeInterval_t(lockedJoint->rankInVelocity(), lockedJoint->numberDof())))
        &&
        fullSolver_.explicitSolver().add(lockedJoint->function(),
            Eigen::RowBlockIndices(),
            Eigen::RowBlockIndices(SizeInterval_t(lockedJoint->rankInConfiguration(), lockedJoint->size())),
            Eigen::ColBlockIndices(),
            Eigen::RowBlockIndices(SizeInterval_t(lockedJoint->rankInVelocity(), lockedJoint->numberDof())));
      if (!added) {
        hppDout (error, "Could not add LockedJoint " << lockedJoint->jointName_);
      }
      minimalSolver_.explicitSolverHasChanged();
      fullSolver_.explicitSolverHasChanged();

      lockedJoints_.push_back (lockedJoint);
      hppDout (info, "add locked joint " << lockedJoint->jointName_
	       << " rank in velocity: " << lockedJoint->rankInVelocity ()
	       << ", size: " << lockedJoint->numberDof ());
      hppDout (info, "Intervals: " << minimalSolver_.explicitSolver().outDers());
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
      // TODO add printer to MatrixBlockIndices
      hppDout (info, minimalSolver_.explicitSolver().outDers());
      os << std::endl;
      return os;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config)
    {
      // return minimalSolver_.isSatisfied (config);
      return fullSolver_.isSatisfied (config);
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config,
				       vector_t& error)
    {
      // error.resize (minimalSolver_.dimension() + minimalSolver_.explicitSolver().outDers().nbIndices());
      // return minimalSolver_.isSatisfied (config, error);
      error.resize (fullSolver_.dimension() + fullSolver_.explicitSolver().outDers().nbIndices());
      return fullSolver_.isSatisfied (config, error);
    }

    vector_t ConfigProjector::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      minimalSolver_.rightHandSideFromInput (config);
      fullSolver_.rightHandSideFromInput (config);

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
      if (!minimalSolver_.rightHandSideFromInput (nm->functionPtr(), config)) {
        throw std::runtime_error ("Function was not found in the solver. This is probably because it is an explicit function and rhs is not supported for this type of function.");
      }
      fullSolver_.rightHandSideFromInput (nm->functionPtr(), config);
    }

    void ConfigProjector::rightHandSideFromConfig (
        const LockedJointPtr_t& lj,
        ConfigurationIn_t config)
    {
      lj->rightHandSideFromConfig (config);
    }

    void ConfigProjector::rightHandSide (const vector_t& small)
    {
      const size_type rhsImplicitSize = minimalSolver_.rightHandSideSize();
      minimalSolver_.rightHandSide (small.head(rhsImplicitSize));
      fullSolver_.rightHandSide (small.head(rhsImplicitSize));

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

    void ConfigProjector::rightHandSide (
        const NumericalConstraintPtr_t& nm,
        vectorIn_t rhs)
    {
      if (!minimalSolver_.rightHandSide (nm->functionPtr(), rhs)) {
        throw std::runtime_error ("Function was not found in the solver. This is probably because it is an explicit function and rhs is not supported for this type of function.");
      }
      fullSolver_.rightHandSide (nm->functionPtr(), rhs);
    }

    void ConfigProjector::rightHandSide (
        const LockedJointPtr_t& lj,
        vectorIn_t rhs)
    {
      if (lj->comparisonType ()->constantRightHandSide ()) {
        lj->rightHandSide (rhs);
      }
    }

    vector_t ConfigProjector::rightHandSide () const
    {
      vector_t small(minimalSolver_.rightHandSideSize() + rhsReducedSize_);

      vector_t rhsImplicit = minimalSolver_.rightHandSide();

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
                               return minimalSolver_.oneStep(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return minimalSolver_.oneStep(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return minimalSolver_.oneStep(config, ls);
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
                               return minimalSolver_.solve(config, ls);
                             }
        case ErrorNormBased: {
                               constraints::lineSearch::ErrorNormBased ls;
                               return minimalSolver_.solve(config, ls);
                             }
        case FixedSequence : {
                               constraints::lineSearch::FixedSequence ls;
                               return minimalSolver_.solve(config, ls);
                             }
      }
      throw std::runtime_error ("Unknow line search type");
      return HybridSolver::MAX_ITERATION_REACHED;
    }
  } // namespace core
} // namespace hpp
