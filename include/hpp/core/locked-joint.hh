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

# include <hpp/pinocchio/joint.hh>

# include <hpp/constraints/affine-function.hh>

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
      /// Copy object and return shared pointer to copy
      virtual EquationPtr_t copy () const;

      /// Create locked joint and return shared pointer
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      static LockedJointPtr_t create (const JointPtr_t& joint,
				      const LiegroupElement& value);

      /// Create partial locked joint (only some degrees of freedom)
      /// \param joint joint that is locked,
      /// \param index first locked degree of freedom in the joint,
      /// \param value of the constant joint partial config, size of value
      ///        determines the number of degrees of freedom locked.
      /// \note valid only for translation joints.
      static LockedJointPtr_t create (const JointPtr_t& joint,
                                      const size_type index,
				      vectorIn_t value);

      /// Create locked degrees of freedom of extra config space
      /// \param robot robot
      /// \param index index of the first  locked extra degree of freedom,
      /// \param value of the locked degrees of freedom, size of value
      ///        determines the number of degrees of freedom locked.
      static LockedJointPtr_t create (const DevicePtr_t& dev,
                                      const size_type index,
				      vectorIn_t value);

      /// Return shared pointer to copy
      /// \param other instance to copy.
      static LockedJointPtr_t createCopy (LockedJointConstPtr_t other);

      /// Get index of locked degree of freedom in robot configuration vector
      std::size_t rankInConfiguration () const;

      /// Get index of locked degree of freedom in robot velocity vector.
      std::size_t rankInVelocity () const;

      /// Get the configuration size of the joint.
      std::size_t configSize () const;

      /// Get number of degrees of freedom of the joint
      std::size_t numberDof () const;

      /// Get configuration space of locked joint
      const LiegroupSpacePtr_t& configSpace () const;

      /// Get the value of the locked joint.
      vectorIn_t value () const;

      /// Set the value of the locked joint.
      void value (vectorIn_t value);

      /// Set the value of the locked joint from a configuration.
      void rightHandSideFromConfig (ConfigurationIn_t config);

      /// Check whether a configuration statisfies the constraint.
      bool isSatisfied (ConfigurationIn_t config);

      /// Check whether a configuration statisfies the constraint.
      ///
      /// \param config input configuration
      /// \retval error difference between configuration of joint read in
      ///         input configuration and locked value.
      bool isSatisfied (ConfigurationIn_t config, vector_t& error);

      DifferentiableFunctionPtr_t function() const
      {
        return function_;
      }

      /// Return shared pointer to joint
      const JointPtr_t& joint ()
      {
        return joint_;
      }
      /// Return the joint name.
      const std::string& jointName () const {
        return jointName_;
      }
      /// Print object in a stream
      std::ostream& print (std::ostream& os) const;
    protected:
      /// Constructor
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      LockedJoint (const JointPtr_t& joint, const LiegroupElement& value);
      /// Constructor of partial locked joint
      /// \param joint joint that is locked,
      /// \param index first locked degree of freedom in the joint,
      /// \param value of the constant joint partial config, size of value
      ///        determines the number of degrees of freedom locked.
      /// \note valid only for translation joints.
      LockedJoint (const JointPtr_t& joint, const size_type index,
          vectorIn_t value);
      /// Constructor of locked degrees of freedom of extra config space
      /// \param robot robot
      /// \param index index of the first  locked extra degree of freedom,
      /// \param value of the locked degrees of freedom, size of value
      ///        determines the number of degrees of freedom locked.
      LockedJoint (const DevicePtr_t& robot, const size_type index,
          vectorIn_t value);
      /// Copy constructor
      LockedJoint (const LockedJoint& other);
      /// Test equality with other instance
      /// \param other object to copy
      /// \param swapAndTest whether we should also check other == this
      virtual bool isEqual (const Equation& other, bool swapAndTest) const;

      void init (const LockedJointPtr_t& self);

    private:
      void initFunction ();

      std::string jointName_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      JointPtr_t joint_;
      LiegroupSpacePtr_t configSpace_;
      constraints::ConstantFunctionPtr_t function_;
      /// Weak pointer to itself
      LockedJointWkPtr_t weak_;

      friend class ConfigProjector;
    }; // class LockedJoint

    /// \}
    inline std::ostream& operator<< (std::ostream& os, const LockedJoint& lj)
    {
      return lj.print (os);
    }
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_LOCKED_JOINT_HH
