// Copyright (c) 2015, LAAS-CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.


#ifndef HPP_CORE_LOCKED_JOINT_HH
# define HPP_CORE_LOCKED_JOINT_HH

# include <hpp/core/equation.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

     /**
     Implementation of Equation specific to locked joint.
     The underlying equation is \f$ q_i (q) = rhs \f$.
     The right hand side of the equation is also called value.
     */
    class HPP_CORE_DLLAPI LockedJoint : public Equation
    {
    public:
      /// Return shared pointer to new object
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      static LockedJointPtr_t create (
          const JointPtr_t& joint, vectorIn_t value,
          ComparisonTypePtr_t comp = ComparisonType::createDefault ());

      /// Return shared pointer to copy
      /// \param other instance to copy.
      static LockedJointPtr_t createCopy (LockedJointConstPtr_t other);

      LockedJointPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }
      /// Get index of locked degree of freedom in robot configuration vector
      std::size_t rankInConfiguration () const;

      /// Get index of locked degree of freedom in robot velocity vector.
      std::size_t rankInVelocity () const;

      /// Get the configuration size of the joint.
      std::size_t size () const;

      /// Get number of degrees of freedom of the joint
      std::size_t numberDof () const;

      /// Get the value of the locked joint.
      vectorIn_t value () const;

      /// Set the value of the locked joint.
      void value (vectorIn_t value);

      /// Set the value of the locked joint from a configuration.
      void rightHandSideFromConfig (ConfigurationIn_t config);

      /// Check whether a configuration statisfies the constraint.
      bool isSatisfied (ConfigurationIn_t config);

      /// Return the joint name.
      const std::string& jointName () const {
        return jointName_;
      }

    protected:
      /// Constructor
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      LockedJoint (const JointPtr_t& joint, vectorIn_t value,
                   ComparisonTypePtr_t comp = ComparisonType::createDefault ());
      /// Copy constructor
      LockedJoint (const LockedJoint& other);

      void init (const LockedJointPtr_t& self);

    private:
      virtual std::ostream& print (std::ostream& os) const;

      std::string jointName_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      size_type numberDof_;
      /// Weak pointer to itself
      LockedJointWkPtr_t weak_;

      friend class ConfigProjector;
      /// Create a fake locked dof for ConfigProjector
      static LockedJointPtr_t create (const DevicePtr_t& robot);

      /// Create a fake locked joint after last joint of robot
      LockedJoint (const DevicePtr_t& robot);
    }; // class LockedJoint
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_LOCKED_JOINT_HH
