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

#include <limits>
#include <Eigen/SVD>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/core/locked-joint.hh>

namespace hpp {
  namespace core {

    std::ostream& operator<< (std::ostream& os, const hpp::statistics::SuccessStatistics& ss)
    {
      return ss.print (os);
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
      squareErrorThreshold_ (errorThreshold * errorThreshold),
      maxIterations_ (maxIterations), rhsReducedSize_ (0),
      toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      dq_ (robot->numberDof ()),
      dqSmall_ (robot->numberDof ()),
      nbNonLockedDofs_ (robot_->numberDof ()),
      nbLockedDofs_ (0),
      squareNorm_(0), weak_ ()
    {
      dq_.setZero ();
    }

    ConfigProjector::ConfigProjector (const ConfigProjector& cp) :
      Constraint (cp), robot_ (cp.robot_), functions_ (cp.functions_),
      passiveDofs_ (cp.passiveDofs_), lockedJoints_ (),
      intervals_ (cp.intervals_),
      squareErrorThreshold_ (cp.squareErrorThreshold_),
      maxIterations_ (cp.maxIterations_),
      rightHandSide_ (cp.rightHandSide_),
      rhsReducedSize_ (cp.rhsReducedSize_), value_ (cp.value_.size ()),
      reducedJacobian_ (cp.reducedJacobian_.rows (),
			cp.reducedJacobian_.cols ()),
      reducedProjector_ (cp.reducedProjector_.rows (),
			 cp.reducedProjector_.cols ()),
      toMinusFrom_ (cp.toMinusFrom_.size ()),
      projMinusFrom_ (cp.projMinusFrom_.size ()),
      dq_ (cp.dq_.size ()), dqSmall_ (cp.dqSmall_.size ()),
      nbNonLockedDofs_ (cp.nbNonLockedDofs_), nbLockedDofs_ (cp.nbLockedDofs_),
      squareNorm_ (cp.squareNorm_), weak_ ()
    {
      dq_.setZero ();
      for (LockedJoints_t::const_iterator it = cp.lockedJoints_.begin ();
	   it != cp.lockedJoints_.end (); ++it) {
	lockedJoints_.push_back ((*it)->copy ());
      }
    }

    ConstraintPtr_t ConfigProjector::copy () const
    {
      return createCopy (weak_.lock ());
    }

    void ConfigProjector::add (const NumericalConstraintPtr_t& nm,
        const SizeIntervals_t& passiveDofs)
    {
      functions_.push_back (nm);
      passiveDofs_.push_back (passiveDofs);
      rhsReducedSize_ += nm->rhsSize ();
      // TODO: no need to recompute intervals.
      computeIntervals ();
      resize ();
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
      std::size_t size = 0;
      for (NumericalConstraints_t::const_iterator it = functions_.begin ();
	   it != functions_.end (); ++it)
        size += (*it)->function().outputSize ();
      nbNonLockedDofs_ = robot_->numberDof () - nbLockedDofs_;
      value_.resize (size);
      rightHandSide_ = vector_t::Zero (size);
      reducedJacobian_.resize (size, nbNonLockedDofs_);
      reducedJacobian_.setConstant (sqrt (-1));
      dqSmall_.resize (nbNonLockedDofs_);
      dq_.setZero ();
      toMinusFromSmall_.resize (nbNonLockedDofs_);
      projMinusFromSmall_.resize (nbNonLockedDofs_);
      projMinusFrom_.setZero ();
      reducedProjector_.resize (nbNonLockedDofs_, nbNonLockedDofs_);
    }

    void ConfigProjector::computeValueAndJacobian
    (ConfigurationIn_t configuration, vectorOut_t value,
     matrixOut_t reducedJacobian)
    {
      size_type row = 0, nbRows = 0;
      IntervalsContainer_t::const_iterator itPassiveDofs
        = passiveDofs_.begin ();
      for (NumericalConstraints_t::iterator it = functions_.begin ();
	   it != functions_.end (); ++it) {
	DifferentiableFunction& f = (*it)->function ();
	vector_t& v = (*it)->value ();
	matrix_t& jacobian = (*it)->jacobian ();
	f (v, configuration);
	f.jacobian (jacobian, configuration);
        (*(*it)->comparisonType ()) (v, jacobian);
	nbRows = f.outputSize ();
	// Copy columns that are not locked
	size_type col = 0;
	value.segment (row, nbRows) = v;
        /// Set the passive DOFs to zero.
	for (SizeIntervals_t::const_iterator it = itPassiveDofs->begin ();
	     it != itPassiveDofs->end (); ++it)
          jacobian.middleCols (it->first, it->second).setZero ();
        /// Copy the non locked DOFs.
	for (SizeIntervals_t::const_iterator itInterval = intervals_.begin ();
	     itInterval != intervals_.end (); ++itInterval) {
	  size_type col0 = itInterval->first;
	  size_type nbCols = itInterval->second;
	  reducedJacobian.block (row, col, nbRows, nbCols) =
	    jacobian.block (0, col0, nbRows, nbCols);
	  col += nbCols;
	}
        row += nbRows;
        ++itPassiveDofs;
      }
      assert (itPassiveDofs == passiveDofs_.end ());
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


    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      hppDout (info, "before projection: " << configuration.transpose ());
      computeLockedDofs (configuration);
      if (functions_.empty ()) return true;
      value_type alpha = .2;
      value_type alphaMax = .95;
      size_type errorDecreased = 3, iter = 0;
      value_type previousSquareNorm =
	std::numeric_limits<value_type>::infinity();
      // Fill value and Jacobian
      computeValueAndJacobian (configuration, value_, reducedJacobian_);
      squareNorm_ = (value_ - rightHandSide_).squaredNorm ();
      while (squareNorm_ > squareErrorThreshold_ && errorDecreased &&
	     iter < maxIterations_) {
	// Linearization of the system of equations
	// 0 - v_{i} = J (q_i) (q_{i+1} - q_{i})
	// q_{i+1} = q_{i} - \alpha_{i} J(q_i)^{+} v_{i}
	// dq = J(q_i)^{+} v_{i}
	Eigen::JacobiSVD <matrix_t> svd (reducedJacobian_,
					 Eigen::ComputeThinU |
					 Eigen::ComputeThinV);
	dqSmall_ = svd.solve(value_ - rightHandSide_);
	uncompressVector (dqSmall_, dq_);
	vector_t v (-alpha * dq_);
	model::integrate (robot_, configuration, v, configuration);
	// Increase alpha towards alphaMax
	computeValueAndJacobian (configuration, value_, reducedJacobian_);
	alpha = alphaMax - .8*(alphaMax - alpha);
	squareNorm_ = (value_ - rightHandSide_).squaredNorm ();
	hppDout (info, "squareNorm = " << squareNorm_);
	--errorDecreased;
	if (squareNorm_ < previousSquareNorm) errorDecreased = 3;
	previousSquareNorm = squareNorm_;
	++iter;
      };
      if (squareNorm_ > squareErrorThreshold_) {
        if (!errorDecreased)
          statistics_.addFailure (REASON_ERROR_INCREASED);
        else
          statistics_.addFailure (REASON_MAX_ITER);
        if (statistics_.nbFailure () > statistics_.nbSuccess ()) {
          hppDout (warning, "Config projector " << name()
              << " seems to fail often.");
          hppDout (warning, statistics_);
        }
      } else {
        statistics_.addSuccess();
      }
      hppDout (info, "number of iterations: " << iter);
      if (squareNorm_ > squareErrorThreshold_) {
	hppDout (info, "Projection failed.");
	return false;
      }
      hppDout (info, "After projection: " << configuration.transpose ());
      return true;
    }

    void ConfigProjector::projectVectorOnKernel (ConfigurationIn_t from,
						 vectorIn_t velocity,
						 vectorOut_t result)
    {
      if (functions_.empty ()) {
        result = velocity;
        return;
      }
      computeValueAndJacobian (from, value_, reducedJacobian_);
      compressVector (velocity, toMinusFromSmall_);
      typedef Eigen::JacobiSVD < matrix_t > Jacobi_t;
      Jacobi_t svd (reducedJacobian_, Eigen::ComputeFullV);
      size_type p = svd.nonzeroSingularValues ();
      size_type n = nbNonLockedDofs_;
      const Eigen::Block <const matrix_t> V1=svd.matrixV ().block (0, 0, n, p);
      reducedProjector_.setIdentity ();
      reducedProjector_ -= V1 * V1.transpose ();
      projMinusFromSmall_ = reducedProjector_ * toMinusFromSmall_;
      uncompressVector (projMinusFromSmall_, result);
    }

    void ConfigProjector::projectOnKernel (ConfigurationIn_t from,
					   ConfigurationIn_t to,
					   ConfigurationOut_t result)
    {
      if (functions_.empty ()) {
        result = to;
        return;
      }
      model::difference (robot_, to, from, toMinusFrom_);
      projectVectorOnKernel (from, toMinusFrom_, projMinusFrom_);
      model::integrate (robot_, from, projMinusFrom_, result);
    }

    void ConfigProjector::computeLockedDofs (ConfigurationOut_t configuration)
    {
      /// LockedDofs are always sorted by their rankInConfiguration.
      for (LockedJoints_t::iterator itLock = lockedJoints_.begin ();
          itLock != lockedJoints_.end (); ++itLock) {
        configuration.segment ((*itLock)->rankInConfiguration (),
            (*itLock)->size ()) = (*itLock)->value ();
      }
    }

    void ConfigProjector::add (const LockedJointPtr_t& lockedJoint)
    {
      // If the same dof is already locked, replace by new value
      for (LockedJoints_t::iterator itLock = lockedJoints_.begin ();
	   itLock != lockedJoints_.end (); ++itLock) {
	if (lockedJoint->rankInVelocity () == (*itLock)->rankInVelocity ()) {
	  *itLock = lockedJoint;
	  return;
	}
      }
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
      if (!lockedJoint->comparisonType ()->constantRightHandSide ())
        rhsReducedSize_ += lockedJoint->rhsSize ();
    }

    void ConfigProjector::addToConstraintSet
    (const ConstraintSetPtr_t& constraintSet)
    {
      if (constraintSet->configProjector_) {
       std::ostringstream oss
         ("Constraint set cannot store more than one config-projector");
       oss << std::endl << *constraintSet;
       throw std::runtime_error (oss.str ());
      }
      constraintSet->configProjector_ = weak_.lock ();
      constraintSet->trivialOrNotConfigProjector_ =
	constraintSet->configProjector_;
      constraintSet->removeFirstElement ();
      Constraint::addToConstraintSet (constraintSet);
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
	os << "[" << it->first << "," << it->second << "], ";
      }
      os << std::endl;
      return os;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config)
    {
      size_type row = 0, nbRows = 0;
      for (NumericalConstraints_t::iterator it = functions_.begin ();
	   it != functions_.end (); ++it) {
	DifferentiableFunction& f = (*it)->function ();
	vector_t& value = (*it)->value ();
	f (value, config);
        (*(*it)->comparisonType ()) (value, (*it)->jacobian ());
	nbRows = f.outputSize ();
	value_.segment (row, nbRows) = value;
	row += nbRows;
      }
      squareNorm_ = (value_ - rightHandSide_).squaredNorm ();
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it )
        if (!(*it)->isSatisfied (config)) {
	  hppDout (info, "locked joint not satisfied.");
	  return false;
	}
      hppDout (info, name () << ": squared error=" << squareNorm_);
      return squareNorm_ < squareErrorThreshold_;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config,
				       vector_t& error)
    {
      size_type row = 0, nbRows = 0;
      bool result = true;
      for (NumericalConstraints_t::iterator it = functions_.begin ();
	   it != functions_.end (); ++it) {
	DifferentiableFunction& f = (*it)->function ();
	vector_t& value = (*it)->value ();
	f (value, config);
        (*(*it)->comparisonType ()) (value, (*it)->jacobian ());
	nbRows = f.outputSize ();
	value_.segment (row, nbRows) = value;
	row += nbRows;
      }
      error = value_ - rightHandSide_;
      squareNorm_ = (value_ - rightHandSide_).squaredNorm ();
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
	   it != lockedJoints_.end (); ++it ) {
	vector_t localError;
	if (!(*it)->isSatisfied (config, localError)) {
	  result = false;
	}
	error.conservativeResize (error.size () + localError.size ());
	error.tail (localError.size ()) = localError;
      }
      return result && squareNorm_ < squareErrorThreshold_;
    }

    vector_t ConfigProjector::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      size_type row = 0, nbRows = 0;
      for (NumericalConstraints_t::iterator it = functions_.begin ();
          it != functions_.end (); ++it) {
        NumericalConstraint& nm = **it;
        const DifferentiableFunction& f = nm.function ();
        nbRows = f.outputSize ();
        nm.rightHandSideFromConfig (config);
        rightHandSide_.segment (row, nm.rhsSize ()) = nm.rightHandSide ();
        row += nbRows;
      }
      // Update other degrees of freedom.
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it )
        (*it)->rightHandSideFromConfig (config);
      return rightHandSide();
    }

    void ConfigProjector::rightHandSide (const vector_t& small)
    {
      size_type row = 0, nbRows = 0, sRow = 0;
      for (NumericalConstraints_t::iterator it = functions_.begin ();
          it != functions_.end (); ++it) {
        NumericalConstraint& nm = **it;
        nbRows = nm.function ().outputSize ();
        nm.rightHandSide (small.segment (sRow, nm.rhsSize ()));
        rightHandSide_.segment (row, nm.rhsSize ()) = nm.rightHandSide ();
        sRow += nm.rhsSize ();
        row += nbRows;
      }
      assert (row == rightHandSide_.size ());
      for (LockedJoints_t::iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it ) {
        LockedJoint& lj = **it;
        if (!lj.comparisonType ()->constantRightHandSide ()) {
          lj.rightHandSide (small.segment (sRow, lj.rhsSize ()));
          sRow += lj.rhsSize ();
        }
      }
      assert (sRow == small.size ());
    }

    void ConfigProjector::updateRightHandSide ()
    {
      size_type row = 0, nbRows = 0, sRow = 0;
      for (NumericalConstraints_t::iterator it = functions_.begin ();
          it != functions_.end (); ++it) {
        NumericalConstraint& nm = **it;
        nbRows = nm.function ().outputSize ();
        rightHandSide_.segment (row, nm.rhsSize ()) = nm.rightHandSide ();
        sRow += nm.rhsSize ();
        row += nbRows;
      }
      assert (row == rightHandSide_.size ());
    }

    vector_t ConfigProjector::rightHandSide () const
    {
      vector_t small(rhsReducedSize_);
      size_type row = 0, nbRows = 0, s = 0;
      for (NumericalConstraints_t::const_iterator it = functions_.begin ();
          it != functions_.end (); ++it) {
        NumericalConstraint& nm = **it;
        nbRows = nm.function ().outputSize ();
        small.segment (s, nm.rhsSize ()) = rightHandSide_.segment (row, nm.rhsSize ());
        s += nm.rhsSize ();
        row += nbRows;
      }
      for (LockedJoints_t::const_iterator it = lockedJoints_.begin ();
          it != lockedJoints_.end (); ++it ) {
        LockedJoint& lj = **it;
        if (!lj.comparisonType ()->constantRightHandSide ()) {
          small.segment (s, lj.rhsSize ()) = lj.rightHandSide ();
          s += lj.rhsSize ();
        }
      }
      assert (s == small.size ());
      return small;
    }
  } // namespace core
} // namespace hpp
