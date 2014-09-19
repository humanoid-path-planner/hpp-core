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
# include <hpp/model/joint.hh>
# include <hpp/core/constraint-set.hh>

namespace hpp {
  namespace core {
     /**
     Constraint that locks a degree of freedom to a constant value.

     For translation and rotation joints the locked degree of freedom
     constraint is built as follows:
     \code
     LockedDofPtr_t lockedDof = LockedDof::create ("lock-dof", joint, value);
     \endcode
     where \c joint is a pointer to a robot joint and \c value is a \c double
     value. However, for JointSO3, it is possible to constrain the rotation
     along one coordinate axis by constraining both quaternion coodinates and
     angular velocity as follows:
     \code
     LockedDofPtr_t l1 = LockedDof::create ("lock-dof1", joint, 0, 1, 0);
     LockedDofPtr_t l1 = LockedDof::create ("lock-dof2", joint, 0, 2, 1);
     \endcode
     This constrains the quaternion value and the velocity vector of the
     joint to be of the forms:
     \f{eqnarray*}
        quat &=& x + 0\ i + 0\ j + w\ k \\
        \omega &=& \omega_z \mathbf{u}_z
     \f}
     */
    class HPP_CORE_DLLAPI LockedDof : public Constraint
    {
    public:
      /// Return shared pointer to new object
      /// \param name name of the constraint,
      /// \param joint joint that is moved by the degree of freedom,
      /// \param value of the degree of freedom,
      /// \param rankInConfig index of the degree of freedom in the joint
      ///        configuration vector (usually 0),
      /// \param rankInVelocity index of the degree of freedom in the joint
      ///        velocity vector (usually 0),
      static LockedDofPtr_t create (const std::string& name,
				    const JointPtr_t& joint,
				    value_type value,
				    size_type rankInConfig = 0,
				    size_type rankInVelocity = 0)
      {
	LockedDof* ptr = new LockedDof (name, joint, value, rankInConfig,
					rankInVelocity);
	LockedDofPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Return shared pointer to new object
      /// \param name name of the constraint,
      /// \param value of the degree of freedom,
      /// \param rankInConfig index of the degree of freedom in the joint
      ///        configuration vector,
      /// \param rankInVelocity index of the degree of freedom in the joint
      ///        velocity vector,
      static LockedDofPtr_t create (const std::string& name,
				    value_type value,
				    size_type rankInConfig,
				    size_type rankInVelocity)
      {
	LockedDof* ptr = new LockedDof (name, value, rankInConfig,
					rankInVelocity);
	LockedDofPtr_t shPtr (ptr);
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

      /// Set the level set parameter of the constraint.
      /// \param config the configuration used to compute the parameter.
      /// \return the parameter.
      vector_t offsetFromConfig (ConfigurationIn_t config)
      {
        if (isParametric_) {
          vector_t ret(1);
          value_ = config[rankInConfiguration_];
          ret[0] = value_;
          return ret;
        }
        return vector_t (0);
      }

      /// Check whether this constraint is parametric.
      /// \return True if parametric.
      bool isParametric () const
      {
        return isParametric_;
      }

      /// Make the constraint parametric or non-parametric.
      /// \param value True if you want it parametric.
      void isParametric (const bool& value)
      {
        isParametric_ = value;
      }

      /// Check whether a configuration statisfies the constraint.
      bool isSatisfied (ConfigurationIn_t config)
      {
	return config [rankInConfiguration_] == value_;
      }

    protected:
      /// Constructor
      /// \param name name of the constraint,
      /// \param joint joint that is moved by the degree of freedom,
      /// \param value of the degree of freedom,
      /// \param rankInConfig index of the degree of freedom in the joint
      ///        configuration vector (usually 0),
      /// \param rankInVelocity index of the degree of freedom in the joint
      ///        velocity vector (usually 0),
      LockedDof (const std::string& name, const JointPtr_t& joint,
		 value_type value, size_type rankInConfig = 0,
		 size_type rankInVelocity = 0) :
	Constraint (name), value_ (value),
	rankInConfiguration_ (joint->rankInConfiguration () + rankInConfig),
        rankInVelocity_ (joint->rankInVelocity () + rankInVelocity),
        isParametric_ (false)
      {
      }
      /// Constructor
      /// \param name name of the constraint,
      /// \param value of the degree of freedom,
      /// \param rankInConfig index of the degree of freedom in the robot
      ///        configuration vector,
      /// \param rankInVelocity index of the degree of freedom in the robot
      ///        velocity vector,
      LockedDof (const std::string& name, value_type value,
		 size_type rankInConfig, size_type rankInVelocity) :
	Constraint (name), value_ (value),
	rankInConfiguration_ (rankInConfig),
        rankInVelocity_ (rankInVelocity),
        isParametric_ (false)
      {
      }
      void init (const LockedDofPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      bool impl_compute (ConfigurationOut_t configuration)
      {
	configuration [rankInConfiguration_] = value_;
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
	constraintSet->addLockedDof (weak_.lock ());
	constraintSet->hasLockedDofs_ = true;
	Constraint::addToConstraintSet (constraintSet);
      }

      value_type value_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      /// Whether this constraint is parametric
      bool isParametric_;
      /// Weak pointer to itself
      LockedDofWkPtr_t weak_;
    }; // class LockedDof
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_LOCKED_DOF_HH
