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

#ifndef HPP_CORE_LOCKED_DOF_HH
# define HPP_CORE_LOCKED_DOF_HH

# include <roboptim/core/differentiable-function.hh>
# include <hpp/core/constraint-set.hh>

namespace hpp {
  namespace core {
    /// Constraint that locks a degree of freedom to a constant value.
    class HPP_CORE_DLLAPI LockedDof : public Constraint
    {
    public:
      /// Return shared pointer to new object
      static LockedDofPtr_t create (const std::string& name, size_type index,
				    value_type value)
      {
	LockedDof* ptr = new LockedDof (name, index, value);
	LockedDofPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Get index of the locked degree of freedom in the configuration vector.
      std::size_t index () const
      {
	return index_;
      }
      /// Get the value of the configuration locked component.
      const value_type& value () const
      {
	return value_;
      }
      /// Get the value of the configuration locked component.
      value_type& value ()
      {
	return value_;
      }
    protected:
      LockedDof (const std::string& name, size_type index, value_type value) :
	Constraint (name), index_ (index), value_ (value)
      {
      }
      void init (const LockedDofPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      bool impl_compute (ConfigurationOut_t configuration)
      {
	configuration [index_] = value_;
	return true;
      }
    private:
      virtual std::ostream& print (std::ostream& os) const
      {
	os << "Locked degree of freedom " << name () << ": index = "
	   << index_ << ", value = " << value_ << std::endl;
	return os;
      }
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet)
      {
	constraintSet->addLockedDof (weak_.lock ());
	constraintSet->hasLockedDofs_ = true;
	Constraint::addToConstraintSet (constraintSet);
      }

      std::size_t index_;
      value_type value_;
      /// Weak pointer to itself
      LockedDofWkPtr_t weak_;
    }; // class LockedDof
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_LOCKED_DOF_HH
