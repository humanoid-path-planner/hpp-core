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

#ifndef HPP_CORE_CONSTRAINT_HH
# define HPP_CORE_CONSTRAINT_HH

# include <ostream>
# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /// Constraint applicable to a robot configuration
    ///
    /// Constraint::apply takes as input a configuration and changes it into
    /// a configuration satisfying the constraint.
    ///
    /// User should define impl_compute in derived classes.
    class HPP_CORE_DLLAPI Constraint
    {
    public:
      /// Function that applies the constraint
      /// \param configuration initial configuration and result
      /// \return true if constraint applied successfully, false if failure.
      bool apply (ConfigurationOut_t configuration);
      /// Get name of constraint
      const std::string& name () const
      {
	return name_;
      }

      /// Check whether a configuration statisfies the constraint.
      ///
      /// \param config the configuration to check
      virtual bool isSatisfied (ConfigurationIn_t config) = 0;

      /// Check whether a configuration statisfies the constraint.
      ///
      /// \param config the configuration to check
      /// \retval error error expressed as a vector. Size and content depends
      ///         on implementations
      virtual bool isSatisfied (ConfigurationIn_t config, vector_t& error) = 0;

      /// return shared pointer to copy
      virtual ConstraintPtr_t copy () const = 0;

      virtual ~Constraint () {};

    protected:
      /// User defined implementation of the constraint.
      virtual bool impl_compute (ConfigurationOut_t configuration) = 0;
      /// Constructor
      Constraint (const std::string& name) : name_ (name), weak_ ()
	{
	}
      Constraint (const Constraint& constraint) : name_ (constraint.name_),
	weak_ ()
	{
	}
      /// Store shared pointer to itself
      void init (const ConstraintPtr_t& self)
      {
	weak_ = self;
      }
    private:
      virtual std::ostream& print (std::ostream& os) const = 0;
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet);

      virtual void addLockedJoint (const LockedJointPtr_t&)
      {
      }

      std::string name_;
      ConstraintWkPtr_t weak_;
      friend class ConstraintSet;
      friend class constraints::LockedJoint;
      friend class ConfigProjector;
      friend std::ostream& operator<< (std::ostream& os, const Constraint&);
    }; // class Constraint
    inline std::ostream& operator<< (std::ostream& os,
				     const Constraint& constraint)
    {
      return constraint.print (os);
    }
    /// \}
  } // namespace core
} // namespace core
#endif // HPP_CORE_CONSTRAINT_HH
