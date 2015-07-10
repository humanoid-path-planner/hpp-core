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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (ConfigProjectorTrivial);
    typedef boost::shared_ptr <ConfigProjectorTrivial>
    ConfigProjectorTrivialPtr_t;
    class ConfigProjectorTrivial : public ConfigProjector
    {
    public:
      static ConfigProjectorTrivialPtr_t create (const DevicePtr_t& robot)
      {
	ConfigProjectorTrivial* ptr = new ConfigProjectorTrivial (robot);
	ConfigProjectorTrivialPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      ConfigProjectorTrivial (const DevicePtr_t& robot)
	: ConfigProjector (robot, "trivial ConfigProjector", 0, 0)
      {
      }
      // Do not copy, return shared pointer to this.
      virtual ConstraintPtr_t copy () const
      {
	return weak_.lock ();
      }
      bool impl_compute (ConfigurationOut_t configuration)
      {
	computeLockedDofs (configuration);
	return true;
      }
      void projectOnKernel (ConfigurationIn_t,
          ConfigurationIn_t, ConfigurationOut_t)
      {}
      /// Check whether a configuration statisfies the constraint.
      virtual bool isSatisfied (ConfigurationIn_t)
      {
	return true;
      }
      virtual bool isSatisfied (ConfigurationIn_t, vector_t& error)
      {
	error.resize (0);
	return true;
      }

      void init (ConfigProjectorTrivialPtr_t weak)
      {
	ConfigProjector::init (weak);
	weak_ = weak;
      }
      ConfigProjectorTrivialWkPtr_t weak_;
    }; // class ConfigProjectorTrivial

    bool ConstraintSet::impl_compute (ConfigurationOut_t configuration)
    {
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->impl_compute (configuration)) return false;
      }
      return true;
    }

    ConstraintPtr_t ConstraintSet::copy () const
    {
      return createCopy (weak_.lock ());
    }

    ConstraintSet::ConstraintSet (const DevicePtr_t& robot,
				  const std::string& name) :
      Constraint (name), constraints_ (), configProjector_ ()
    {
      trivialOrNotConfigProjector_ = ConfigProjectorTrivial::create (robot);
      constraints_.push_back (trivialOrNotConfigProjector_);
    }

    ConstraintSet::ConstraintSet (const ConstraintSet& other) :
      Constraint (other), constraints_ (), configProjector_ ()
    {
      for (Constraints_t::const_iterator it = other.constraints_.begin ();
	   it != other.constraints_.end (); ++it) {
	constraints_.push_back ((*it)->copy ());
      }
      if (other.configProjector_) {
	configProjector_ = HPP_STATIC_PTR_CAST
	  (ConfigProjector, other.configProjector_->copy ());
      } else {
	assert (HPP_DYNAMIC_PTR_CAST (ConfigProjectorTrivial,
				      constraints_ [0]));
	trivialOrNotConfigProjector_ = HPP_STATIC_PTR_CAST
	  (ConfigProjector, constraints_ [0]);
      }
    }

    void ConstraintSet::removeFirstElement ()
    {
      constraints_.pop_front ();
    }

    bool ConstraintSet::isSatisfied (ConfigurationIn_t configuration)
    {
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->isSatisfied (configuration)) {
	  return false;
	}
      }
      return true;
    }

    bool ConstraintSet::isSatisfied (ConfigurationIn_t configuration,
				     vector_t& error)
    {
      bool result = true;
      error.resize (0);
      vector_t localError;
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->isSatisfied (configuration, localError)) {
	  error.conservativeResize (error.size () + localError.size ());
	  error.tail (localError.size ()) = localError;
	  result = false;
	}
      }
      return result;
    }

    size_type ConstraintSet::numberNonLockedDof () const
    {
      return trivialOrNotConfigProjector_->numberNonLockedDof ();
    }

    void ConstraintSet::compressVector (vectorIn_t normal,
					vectorOut_t small) const
    {
      trivialOrNotConfigProjector_->compressVector (normal, small);
    }

    void ConstraintSet::uncompressVector (vectorIn_t small,
					  vectorOut_t normal) const
    {
      trivialOrNotConfigProjector_->uncompressVector (small, normal);
    }

    void ConstraintSet::compressMatrix (matrixIn_t normal, matrixOut_t small,
					bool rows) const
    {
      trivialOrNotConfigProjector_->compressMatrix (normal, small, rows);
    }

    void ConstraintSet::uncompressMatrix (matrixIn_t small,
					  matrixOut_t normal,
					  bool rows) const
    {
      trivialOrNotConfigProjector_->uncompressMatrix (small, normal, rows);
    }

  } // namespace core
} // namespace core
