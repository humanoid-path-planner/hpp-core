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
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/differentiable-function.hh>
#include <hpp/core/locked-dof.hh>

namespace hpp {
  namespace core {
    //using boost::fusion::result_of::at;
    bool operator< (const LockedDofPtr_t& l1, const LockedDofPtr_t& l2)
    {
      return l1->index () < l2->index ();
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

    ConfigProjector::ConfigProjector (const DevicePtr_t& robot,
				      const std::string& name,
				      value_type errorThreshold,
				      size_type maxIterations) :
      Constraint (name), robot_ (robot), constraints_ (),
      squareErrorThreshold_ (errorThreshold * errorThreshold),
      maxIterations_ (maxIterations), toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      dq_ (robot->numberDof ()),
      dqSmall_ (robot->numberDof ()),
      nbNonLockedDofs_ (robot_->numberDof ())
    {
    }

    void ConfigProjector::addConstraint
    (const DifferentiableFunctionPtr_t& constraint)
    {
      vector_t value (constraint->outputSize ());
      matrix_t jacobian (constraint->outputSize (),
			 robot_->numberDof ());
      hppDout (info, "constraint->getName () = " << constraint->name ());
      hppDout (info, "constraint->inputSize () = " <<
	       constraint->inputSize ());
      hppDout (info, "constraint->outputSize () = " <<
	       constraint->outputSize ());
      hppDout (info, "value.rows () = " << value.rows ());
      hppDout (info, "jacobian.rows () = " << jacobian.rows ());
      hppDout (info, "jacobian.cols () = " << jacobian.cols ());
      constraints_.push_back (FunctionValueAndJacobian_t (constraint, value,
							  jacobian));
      computeIntervals ();
      resize ();
    }

    void ConfigProjector::computeIntervals ()
    {
      intervals_.clear ();
      std::pair < size_type, size_type > interval;
      int latestIndex=-1;
      size_type size;
      lockedDofs_.sort ();
      // temporarily add an element at the end of the list.
      lockedDofs_.push_back (LockedDof::create ("temporary",
						robot_->numberDof (), 0));
      for (LockedDofs_t::const_iterator itLocked = lockedDofs_.begin ();
	   itLocked != lockedDofs_.end (); itLocked++) {
	int index = (*itLocked)->index ();
	size = (index - latestIndex) - 1;
	if (size > 0) {
	  interval.first = latestIndex + 1;
	  interval.second = size;
	  intervals_.push_back (interval);
	}
      }
      // Remove temporary element.
      lockedDofs_.pop_back ();
    }

    void ConfigProjector::resize ()
    {
      std::size_t size = 0;
      for (NumericalConstraints_t::const_iterator itConstraint =
	     constraints_.begin ();
	   itConstraint != constraints_.end (); itConstraint ++) {
	FunctionValueAndJacobian_t fvj = (*itConstraint);
	size += fvj.function->outputSize ();
      }
      nbNonLockedDofs_ = robot_->numberDof () - lockedDofs_.size ();
      value_.resize (size);
      reducedJacobian_.resize (size, nbNonLockedDofs_);
      dqSmall_.resize (nbNonLockedDofs_);
      dq_.fill (0.);
      toMinusFromSmall_.resize (nbNonLockedDofs_);
      projMinusFromSmall_.resize (nbNonLockedDofs_);
      reducedProjector_.resize (nbNonLockedDofs_, nbNonLockedDofs_);
    }

    void ConfigProjector::computeValueAndJacobian
    (const Configuration_t& configuration)
    {
      size_type row = 0, nbRows = 0;
      for (NumericalConstraints_t::iterator itConstraint =
	     constraints_.begin ();
	   itConstraint != constraints_.end (); itConstraint ++) {
	DifferentiableFunction& f = *(itConstraint->function);
	vector_t& value = itConstraint->value;
	matrix_t& jacobian = itConstraint->jacobian;
	f (value, configuration);
	f.jacobian (jacobian, configuration);
	hppDout (info, "Function " << f.name ());
	hppDout (info, "value: " << std::endl << value);
	hppDout (info, "Jacobian: " << std::endl << jacobian);
	nbRows = f.outputSize ();
	// Copy columns that are not locked
	size_type col = 0;
	value_.segment (row, nbRows) = value;
	for (Intervals_t::const_iterator itInterval = intervals_.begin ();
	     itInterval != intervals_.end (); itInterval ++) {
	  size_type col0 = itInterval->first;
	  size_type nbCols = itInterval->second;
	  reducedJacobian_.block (row, col, nbRows, nbCols) =
	    jacobian.block (0, col0, nbRows, nbCols);
	  col += nbCols;
	}
	row += nbRows;
      }
      hppDout (info, "Constraint value: " << value_);
      hppDout (info, "Reduced Jacobian: " << reducedJacobian_);
    }

    /// Convert vector of non locked degrees of freedom to vector of
    /// all degrees of freedom
    void ConfigProjector::smallToNormal (const vector_t& small,
					 vector_t& normal)
    {
      assert (small.size () + lockedDofs_.size () == robot_->numberDof ());
      assert (normal.size () - robot_->numberDof () == 0);
      size_type col = 0;
      for (Intervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); itInterval ++) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	normal.segment (col0, nbCols) = small.segment (col, nbCols);
	col += itInterval->second;
      }
    }

    void ConfigProjector::normalToSmall (const vector_t& normal,
					 vector_t& small)
    {
      assert (small.size () + lockedDofs_.size () == robot_->numberDof ());
      assert (abs (normal.size ()) - robot_->numberDof () == 0);
      size_type col = 0;
      for (Intervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); itInterval ++) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	small.segment (col, nbCols) = normal.segment (col0, nbCols);
	col += itInterval->second;
      }
    }

    bool ConfigProjector::impl_compute (Configuration_t& configuration)
    {
      hppDout (info, "before projection: " << configuration.transpose ());
      computeLockedDofs (configuration);
      value_type alpha = .2;
      value_type alphaMax = .95;
      size_type errorDecreased = 3, iter = 0;
      value_type previousSquareNorm =
	std::numeric_limits<value_type>::infinity();
      value_type squareNorm;
      // Fill value and Jacobian
      computeValueAndJacobian (configuration);
      squareNorm = value_.squaredNorm ();
      while (squareNorm > squareErrorThreshold_ && errorDecreased &&
	     iter < maxIterations_) {
	// Linearization of the system of equations
	// 0 - v_{i} = J (q_i) (q_{i+1} - q_{i})
	// q_{i+1} = q_{i} - \alpha_{i} J(q_i)^{+} v_{i}
	// dq = J(q_i)^{+} v_{i}
	Eigen::JacobiSVD <matrix_t> svd (reducedJacobian_,
					 Eigen::ComputeThinU |
					 Eigen::ComputeThinV);
	hppDout (info, "singular values: " << svd.singularValues ());
	dqSmall_ = svd.solve(value_);
	hppDout (info, "dqSmall_ = " << dqSmall_);
	smallToNormal (dqSmall_, dq_);
	model::integrate (robot_, configuration, -alpha * dq_, configuration);
	// Increase alpha towards alphaMax
	alpha = alphaMax - .8*(alphaMax - alpha);
	squareNorm = value_.squaredNorm ();
	hppDout (info, "squareNorm = " << squareNorm);
	--errorDecreased;
	if (squareNorm < previousSquareNorm) errorDecreased = 3;
	previousSquareNorm = squareNorm;
	++iter;
	computeValueAndJacobian (configuration);
      };
      if (squareNorm > squareErrorThreshold_) {
	hppDout (info, "Projection failed.");
	return false;
      }
      hppDout (info, "After projection: " << configuration.transpose ());
      return true;
    }

    void ConfigProjector::projectOnKernel (const Configuration_t& from,
					   const Configuration_t& to,
					   Configuration_t& result)
    {
      computeValueAndJacobian (from);
      model::difference (robot_, to, from, toMinusFrom_);
      normalToSmall (toMinusFrom_, toMinusFromSmall_);
      typedef Eigen::JacobiSVD < matrix_t > Jacobi_t;
      Jacobi_t svd (reducedJacobian_, Eigen::ComputeFullV);
      size_type p = svd.nonzeroSingularValues ();
      size_type n = nbNonLockedDofs_;
      const Eigen::Block <const matrix_t> V1=svd.matrixV ().block (0, 0, n, p);
      reducedProjector_.setIdentity ();
      reducedProjector_ -= V1 * V1.transpose ();
      projMinusFromSmall_ = reducedProjector_ * toMinusFromSmall_;
      smallToNormal (projMinusFromSmall_, projMinusFrom_);
      model::integrate (robot_, from, projMinusFrom_, result);
    }

    void ConfigProjector::computeLockedDofs (Configuration_t& configuration)
    {
      for (LockedDofs_t::iterator itLock = lockedDofs_.begin ();
	   itLock != lockedDofs_.end (); itLock ++) {
	configuration [(*itLock)->index ()] = (*itLock)->value ();
      }
    }

    void ConfigProjector::addLockedDof (const LockedDofPtr_t& lockedDof)
    {
      // If the same dof is already locked, replace by new value
      for (LockedDofs_t::iterator itLock = lockedDofs_.begin ();
	   itLock != lockedDofs_.end (); itLock ++) {
	if (lockedDof->index () == (*itLock)->index ()) {
	  *itLock = lockedDof;
	  return;
	}
      }
      lockedDofs_.push_back (lockedDof);
      computeIntervals ();
      resize ();
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
      if (constraintSet->hasLockedDofs ()) {
	std::ostringstream oss
	  ("Locked dofs should be inserted after config-projector");
	oss << std::endl << *constraintSet;
	throw std::runtime_error (oss.str ());
      }
      constraintSet->configProjector_ = weak_.lock ();
      constraintSet->removeFirstElement ();
      Constraint::addToConstraintSet (constraintSet);
    }

    std::ostream& ConfigProjector::print (std::ostream& os) const
    {
      os << "Config projector: " << name () << ", contains" << std::endl;
      for (NumericalConstraints_t::const_iterator it = constraints_.begin ();
	   it != constraints_.end (); it++) {
	const DifferentiableFunction& f (*(it->function));
	os << f << std::endl;
      }
      return os;
    }

  } // namespace core
} // namespace hpp
