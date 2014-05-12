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

#ifndef HPP_CORE_CONSTRAINT_SET_HH
# define HPP_CORE_CONSTRAINT_SET_HH

# include <deque>
# include <hpp/core/constraint.hh>

namespace hpp {
  namespace core {
    /// Set of constraints applicable to a robot configuration
    ///
    /// \warning If the set is to contain a ConfigProjector and several
    /// LockedDof instances, the configProjector should be inserted first
    /// since following numerical projections might affect locked degrees of
    /// freedom.
    class HPP_CORE_DLLAPI ConstraintSet : public Constraint
    {
    public:
      /// Return shared pointer to new object
      static ConstraintSetPtr_t create (const DevicePtr_t& robot,
					const std::string& name)
      {
	ConstraintSet* ptr = new ConstraintSet (robot, name);
	ConstraintSetPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Add a constraint to the set
      void addConstraint (const ConstraintPtr_t& constraint)
      {
	constraint->addToConstraintSet (weak_.lock ());
      }

      /// Whether constraint set contains constraints of type LockedDof.
      bool hasLockedDofs () const
      {
	return hasLockedDofs_;
      }
      /// Return pointer to config projector if any
      const ConfigProjectorPtr_t& configProjector () const
      {
	return configProjector_;
      }
    protected:
      typedef std::deque <ConstraintPtr_t> Constraints_t;
      ConstraintSet (const DevicePtr_t& robot, const std::string& name);
      /// Store weak pointer to itself.
      void init (const ConstraintSetPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      virtual bool impl_compute (ConfigurationOut_t configuration);
    private:
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet)
      {
	for (Constraints_t::iterator itConstraint = constraints_.begin ();
	     itConstraint != constraints_.end (); itConstraint ++) {
	  (*itConstraint)->addToConstraintSet (constraintSet);
	}
      }
      virtual void addLockedDof (const LockedDofPtr_t& lockedDof)
      {
	for (Constraints_t::iterator itConstraint = constraints_.begin ();
	     itConstraint != constraints_.end (); itConstraint ++) {
	  (*itConstraint)->addLockedDof (lockedDof);
	}
      }

      void removeFirstElement ();

      virtual std::ostream& print (std::ostream& os) const
      {
	os << "Constraint set " << name () << ", contains" << std::endl;
	for (Constraints_t::const_iterator itConstraint = constraints_.begin ();
	     itConstraint != constraints_.end (); itConstraint++) {
	  os << "  " << **itConstraint << std::endl;
	}
	return os;
      }

      Constraints_t constraints_;
      ConfigProjectorPtr_t configProjector_;
      bool hasLockedDofs_;
      ConstraintSetWkPtr_t weak_;

      friend class LockedDof;
      friend class Constraint;
      friend class ConfigProjector;
    }; // class ConstraintSet
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_CONSTRAINT_SET_HH
