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

#ifndef HPP_CORE_LOCKED_JOINT_HH
# define HPP_CORE_LOCKED_JOINT_HH

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/core/constraint-set.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

     /**
     Constraint that locks a joint to a constant value.
     */
    class HPP_CORE_DLLAPI LockedJoint : public Constraint
    {
    public:
      /// Return shared pointer to new object
      /// \param name name of the constraint,
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      static LockedJointPtr_t create (const std::string& name,
				      const JointPtr_t& joint,
				      vectorIn_t value)
      {
	LockedJoint* ptr = new LockedJoint (name, joint, value);
	LockedJointPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Get index of locked degree of freedom in robot configuration vector
      std::size_t rankInConfiguration () const
      {
	return rankInConfiguration_;
      }
      /// Get index of locked degree of freedom in robot velocity vector.
      std::size_t rankInVelocity () const
      {
	return rankInVelocity_;
      }
      /// Get number of degrees of freedom of the joint
      std::size_t numberDof () const
      {
	return numberDof_;
      }
      /// Get the value of the locked joint.
      const vector_t& value () const
      {
	return value_;
      }
      /// Set the value of the locked joint.
      void value (vectorIn_t value)
      {
	value_ = value;
      }
      /// Set the value of the locked joint from a configuration.
      void valueFromFromConfig (ConfigurationIn_t config)
      {
	value_ = config.segment (rankInConfiguration_, value_.size ());
      }
      /// Check whether a configuration statisfies the constraint.
      bool isSatisfied (ConfigurationIn_t config)
      {
	return config.segment (rankInConfiguration_, value_.size ()) == value_;
      }

    protected:
      /// Constructor
      /// \param name name of the constraint,
      /// \param joint joint that is locked,
      /// \param value of the constant joint config,
      LockedJoint (const std::string& name, const JointPtr_t& joint,
		   vectorIn_t value) :
	Constraint (name), value_ (value),
	rankInConfiguration_ (joint->rankInConfiguration ()),
        rankInVelocity_ (joint->rankInVelocity ()),
	numberDof_ (joint->numberDof ())
      {
	assert (value_.size () == joint->configSize ());
      }
      void init (const LockedJointPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      bool impl_compute (ConfigurationOut_t configuration)
      {
	configuration.segment (rankInConfiguration_, value_.size ()) = value_;
	return true;
      }
    private:
      virtual std::ostream& print (std::ostream& os) const
      {
	os << "Locked degree of freedom " << name ()
	   << ", value = " << value_
	   << ": rank in configuration = " << rankInConfiguration_
	   << ": rank in velocity = " << rankInVelocity_
	   << std::endl;
	return os;
      }
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet)
      {
	constraintSet->addLockedJoint (weak_.lock ());
	constraintSet->hasLockedDofs_ = true;
      }

      vector_t value_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      size_type numberDof_;
      /// Weak pointer to itself
      LockedJointWkPtr_t weak_;

      friend class ConfigProjector;
      /// Create a fake locked dof for ConfigProjector
      static LockedJointPtr_t create (const std::string & name,
				      const DevicePtr_t& robot)
      {
	LockedJoint* ptr = new LockedJoint (name, robot);
	LockedJointPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create a fake locked joint after last joint of robot
      LockedJoint (const std::string& name, const DevicePtr_t& robot) :
	Constraint (name), value_ (1),
	rankInConfiguration_ (robot->configSize ()),
	rankInVelocity_ (robot->numberDof ()), numberDof_ (0)
	  {
	  }
    }; // class LockedJoint
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_LOCKED_JOINT_HH
