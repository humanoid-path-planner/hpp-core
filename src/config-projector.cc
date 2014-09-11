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
#include <hpp/model/joint.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/differentiable-function.hh>
#include <hpp/core/locked-dof.hh>

namespace hpp {
  namespace core {

    std::ostream& operator<< (std::ostream& os, const hpp::statistics::SuccessStatistics& ss)
    {
      return ss.print (os);
    }
    HPP_DEFINE_REASON_FAILURE (REASON_MAX_ITER, "Max Iterations reached");
    HPP_DEFINE_REASON_FAILURE (REASON_ERROR_INCREASED, "Error increased");

    //using boost::fusion::result_of::at;
    bool operator< (const LockedDofPtr_t& l1, const LockedDofPtr_t& l2)
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

    ConfigProjector::ConfigProjector (const DevicePtr_t& robot,
				      const std::string& name,
				      value_type errorThreshold,
				      size_type maxIterations) :
      Constraint (name), robot_ (robot), constraints_ (),
      lockedDofs_ (), allSO3ranks_ (), lockedSO3ranks_ (),
      squareErrorThreshold_ (errorThreshold * errorThreshold),
      maxIterations_ (maxIterations), toMinusFrom_ (robot->numberDof ()),
      projMinusFrom_ (robot->numberDof ()),
      dq_ (robot->numberDof ()),
      dqSmall_ (robot->numberDof ()),
      nbNonLockedDofs_ (robot_->numberDof ()),
      squareNorm_(0), weak_ ()
    {
      // Initialize the list of SO3 ranks in the device.
      JointVector_t joints = robot->getJointVector ();
      std::pair < size_type, size_type > rank;
      for (JointVector_t::iterator j = joints.begin();
          j!= joints.end(); j++)
        if (dynamic_cast<model::JointSO3*>(*j) != 0) {
          rank.first = (*j)->rankInConfiguration ();
          rank.second = (*j)->rankInVelocity ();
          allSO3ranks_.push_back (rank);
        }
    }

    void ConfigProjector::addConstraint
    (const DifferentiableFunctionPtr_t& constraint, InequalityPtr_t comp)
    {
      vector_t value (constraint->outputSize ());
      matrix_t jacobian (constraint->outputSize (),
			 robot_->numberDof ());
      constraints_.push_back (FunctionValueAndJacobian_t (constraint, value,
							  jacobian, comp));
      computeIntervals ();
      resize ();
    }

    void ConfigProjector::computeIntervals ()
    {
      intervals_.clear ();
      lockedSO3ranks_.clear();
      lockedSO3values_.clear();
      std::vector <bool> locked(robot_->numberDof(), false);
      std::pair < size_type, size_type > interval;
      int latestIndex=-1;
      size_type size;
      lockedDofs_.sort ();
      // temporarily add an element at the end of the list.
      lockedDofs_.push_back (LockedDof::create ("temporary", 0.,
						robot_->configSize (),
						robot_->numberDof ()));
      for (LockedDofs_t::const_iterator itLocked = lockedDofs_.begin ();
	   itLocked != lockedDofs_.end (); itLocked++) {
	int index = (*itLocked)->rankInVelocity ();
        locked[index] = true;
	size = (index - latestIndex) - 1;
	if (size > 0) {
	  interval.first = latestIndex + 1;
	  interval.second = size;
	  intervals_.push_back (interval);
	}
	latestIndex = index;
      }
      for (Intervals_t::iterator rank = allSO3ranks_.begin();
          rank != allSO3ranks_.end(); rank++) {
        int index = rank->second;
        if (locked[index] && locked[index+1] && locked[index+2]) {
          lockedSO3ranks_.push_back(*rank);
          // TODO: This value corresponds to the first element of the
          // quaternion of the locked SO3 joint. The user should be able
          // to initialize it somehow. It can be initialize using offsetFromConfig
          lockedSO3values_.push_back(0);
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
      offset_ = vector_t::Zero (size);
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
    (ConfigurationIn_t configuration)
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
        (*(itConstraint->comp))(value, jacobian);
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
    }

    /// Convert vector of non locked degrees of freedom to vector of
    /// all degrees of freedom
    void ConfigProjector::smallToNormal (vectorIn_t small,
					 vectorOut_t normal)
    {
      assert (small.size () + (size_type) lockedDofs_.size () ==
	      robot_->numberDof ());
      assert (normal.size () == robot_->numberDof ());
      size_type col = 0;
      for (Intervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); itInterval ++) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	normal.segment (col0, nbCols) = small.segment (col, nbCols);
	col += itInterval->second;
      }
    }

    void ConfigProjector::normalToSmall (vectorIn_t normal,
					 vectorOut_t small)
    {
      assert (small.size () + (size_type) lockedDofs_.size () ==
	      robot_->numberDof ());
      assert (normal.size () == robot_->numberDof ());
      size_type col = 0;
      for (Intervals_t::const_iterator itInterval = intervals_.begin ();
	   itInterval != intervals_.end (); itInterval ++) {
	size_type col0 = itInterval->first;
	size_type nbCols = itInterval->second;
	small.segment (col, nbCols) = normal.segment (col0, nbCols);
	col += itInterval->second;
      }
    }

    bool ConfigProjector::impl_compute (ConfigurationOut_t configuration)
    {
      hppDout (info, "before projection: " << configuration.transpose ());
      computeLockedDofs (configuration);
      value_type alpha = .2;
      value_type alphaMax = .95;
      size_type errorDecreased = 3, iter = 0;
      value_type previousSquareNorm =
	std::numeric_limits<value_type>::infinity();
      // Fill value and Jacobian
      computeValueAndJacobian (configuration);
      squareNorm_ = (value_ - offset_).squaredNorm ();
      while (squareNorm_ > squareErrorThreshold_ && errorDecreased &&
	     iter < maxIterations_) {
	// Linearization of the system of equations
	// 0 - v_{i} = J (q_i) (q_{i+1} - q_{i})
	// q_{i+1} = q_{i} - \alpha_{i} J(q_i)^{+} v_{i}
	// dq = J(q_i)^{+} v_{i}
	Eigen::JacobiSVD <matrix_t> svd (reducedJacobian_,
					 Eigen::ComputeThinU |
					 Eigen::ComputeThinV);
	dqSmall_ = svd.solve(value_ - offset_);
	smallToNormal (dqSmall_, dq_);
	vector_t v (-alpha * dq_);
	model::integrate (robot_, configuration, v, configuration);
	// Increase alpha towards alphaMax
	computeValueAndJacobian (configuration);
	alpha = alphaMax - .8*(alphaMax - alpha);
	squareNorm_ = (value_ - offset_).squaredNorm ();
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

    void ConfigProjector::projectOnKernel (ConfigurationIn_t from,
					   ConfigurationIn_t to,
					   ConfigurationOut_t result)
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

    void ConfigProjector::computeLockedDofs (ConfigurationOut_t configuration)
    {
      // Normalize quaternion if its 3 DOFS are locked.
      size_t rankSO3 = configuration.size(),
                iSO3 = 0;
      if (iSO3 < lockedSO3ranks_.size())
        rankSO3 = lockedSO3ranks_[iSO3].first;
      /// LockedDofs are always sorted by their rankInConfiguration.
      for (LockedDofs_t::iterator itLock = lockedDofs_.begin ();
	   itLock != lockedDofs_.end (); itLock ++) {
	configuration [(*itLock)->rankInConfiguration ()] = (*itLock)->value ();
        if (rankSO3 + 3 == (*itLock)->rankInConfiguration ()) {
          // Normalize
          value_type w = sqrt(1 - configuration.segment (rankSO3 + 1, 3).squaredNorm ());
          if (lockedSO3values_[iSO3] >= 0)
            configuration[rankSO3] = w;
          else
            configuration[rankSO3] = -w;
          assert(0.99 < configuration.segment (rankSO3, 4).squaredNorm ());
          assert(configuration.segment (rankSO3, 4).squaredNorm () < 1.01);
          iSO3++;
          if (iSO3 < lockedSO3ranks_.size())
            rankSO3 = lockedSO3ranks_[iSO3].first;
        }
      }
    }

    void ConfigProjector::addLockedDof (const LockedDofPtr_t& lockedDof)
    {
      // If the same dof is already locked, replace by new value
      for (LockedDofs_t::iterator itLock = lockedDofs_.begin ();
	   itLock != lockedDofs_.end (); itLock ++) {
	if (lockedDof->rankInVelocity () == (*itLock)->rankInVelocity ()) {
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
	os << "    " << f << std::endl;
      }
      return os;
    }

    bool ConfigProjector::isSatisfied (ConfigurationIn_t config)
    {
      size_type row = 0, nbRows = 0;
      for (NumericalConstraints_t::iterator itConstraint =
	     constraints_.begin ();
	   itConstraint != constraints_.end (); itConstraint ++) {
	DifferentiableFunction& f = *(itConstraint->function);
	vector_t& value = itConstraint->value;
	f (value, config);
	nbRows = f.outputSize ();
	value_.segment (row, nbRows) = value;
	row += nbRows;
      }
      return (value_ - offset_).squaredNorm () < squareErrorThreshold_;
    }

    vector_t ConfigProjector::offsetFromConfig (ConfigurationIn_t config)
    {
      size_type row = 0, nbRows = 0;
      for (NumericalConstraints_t::iterator itConstraint =
          constraints_.begin ();
          itConstraint != constraints_.end (); itConstraint ++) {
        vector_t value = vector_t::Zero (itConstraint->value.size ());
        DifferentiableFunction& f = *(itConstraint->function);
        if (f.isParametric ()) {
          f (value, config);
        }
        nbRows = f.outputSize ();
        offset_.segment (row, nbRows) = value;
        row += nbRows;
      }
      for (size_t iSO3 = 0; iSO3 < lockedSO3ranks_.size (); iSO3++)
        lockedSO3values_[iSO3] = config[lockedSO3ranks_[iSO3].first];
      return offset();
    }

    void ConfigProjector::offset (const vector_t& small)
    {
      size_type row = 0, nbRows = 0, s = 0;
      for (NumericalConstraints_t::iterator itConstraint =
          constraints_.begin ();
          itConstraint != constraints_.end (); itConstraint ++) {
        DifferentiableFunction& f = *(itConstraint->function);
        nbRows = f.outputSize ();
        if (f.isParametric ()) {
          offset_.segment (row, nbRows) = small.segment (s, nbRows);
          s += nbRows;
        }
        row += nbRows;
      }
      assert (row == offset_.size ());
    }

    vector_t ConfigProjector::offset () const
    {
      vector_t small;
      size_type row = 0, nbRows = 0, s = 0;
      for (NumericalConstraints_t::const_iterator itConstraint =
          constraints_.begin ();
          itConstraint != constraints_.end (); itConstraint ++) {
        DifferentiableFunction& f = *(itConstraint->function);
        nbRows = f.outputSize ();
        if (f.isParametric ()) {
          small.conservativeResize (s + nbRows);
          small.segment (s, nbRows) = offset_.segment (row, nbRows);
          s += nbRows;
        }
        row += nbRows;
      }
      return small;
    }
  } // namespace core
} // namespace hpp
