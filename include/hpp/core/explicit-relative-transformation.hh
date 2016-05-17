// Copyright (c) 2015, LAAS-CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_EXPLICIT_RELATIVE_TRANSFORM_HH
# define HPP_CORE_EXPLICIT_RELATIVE_TRANSFORM_HH

# include <hpp/fcl/math/transform.h>
# include <hpp/constraints/generic-transformation.hh>
# include <hpp/core/explicit-numerical-constraint.hh>

namespace hpp {
  namespace core {
    using constraints::RelativeTransformation;
    using constraints::RelativeTransformationPtr_t;

    /// \addtogroup constraints
    /// \{

    /// Relative transformation as an explicit constraint
    ///
    /// When the positions of two joints are constrained by a full
    /// transformation constraint, if the second joint is hold by a freeflyer
    /// sequence (R3 x SO(3)), the position of this latter joint can be
    /// explicitely expressed with respect to the position of the first joint.
    ///
    /// This class provides this expression through method solve.
    ///
    /// When considered as an implicit constraint, for instance if this
    /// constraint is grouped with other constraints, the classical
    //  constraint::RelativeTransformation class is used.
    class HPP_CORE_DLLAPI ExplicitRelativeTransformation :
      public ExplicitNumericalConstraint
    {
    public:
      /// Copy object and return shared pointer to copy
      virtual EquationPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }
      /// Return a shared pointer to a new instance
      ///
      /// \param name the name of the constraints,
      /// \param robot the robot the constraints is applied to,
      /// \param joint1 input joint
      /// \param joint2 output joint: position of this joint is computed with
      ///        respect to joint1 position
      /// \param frame1 position of a fixed frame in joint 1,
      /// \param frame2 position of a fixed frame in joint 2,
      static ExplicitRelativeTransformationPtr_t create
	(const std::string& name, const DevicePtr_t& robot,
	 const JointPtr_t& joint1, const JointPtr_t& joint2,
	 const Transform3f& frame1, const Transform3f& frame2)
      {
	ExplicitRelativeTransformation* ptr =
	  new ExplicitRelativeTransformation (name, robot, joint1, joint2,
					      frame1, frame2);
	ExplicitRelativeTransformationPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      static ExplicitRelativeTransformationPtr_t createCopy
	(const ExplicitRelativeTransformationPtr_t& other)
      {
	ExplicitRelativeTransformation* ptr = new ExplicitRelativeTransformation
	  (*other);
	ExplicitRelativeTransformationPtr_t shPtr (ptr);
	ExplicitRelativeTransformationWkPtr_t wkPtr (shPtr);
	ptr->init (wkPtr);
	return shPtr;
    }

      /// Solve constraint
      ///
      /// Compute output with respect to input.
      /// \param configuration input and output configuration
      virtual void solve (ConfigurationOut_t configuration)
      {
	using fcl::inverse;
	robot_->currentConfiguration (configuration);
	robot_->computeForwardKinematics ();
	// J1 * M1/J1 = J2 * M2/J2
	// J2 = J1 * M1/J1 * M2/J2^{-1}
	// J2 = J2_{parent} * T
	// T = J2_{parent}^{-1} * J2
	// T = J2_{parent}^{-1} * J1 * F1/J1 * F2/J2^{-1}
	freeflyerPose_ =
	  inverse (parentJoint_->currentTransformation ()) *
	  relativeTransformation_->joint1 ()->currentTransformation () *
	  relativeTransformation_->frame1InJoint1 () *
	  inverse (relativeTransformation_->frame2InJoint2 ());
	configuration [index_ + 0] = freeflyerPose_.getTranslation () [0];
	configuration [index_ + 1] = freeflyerPose_.getTranslation () [1];
	configuration [index_ + 2] = freeflyerPose_.getTranslation () [2];
	configuration [index_ + 3] = freeflyerPose_.getQuatRotation () [0];
	configuration [index_ + 4] = freeflyerPose_.getQuatRotation () [1];
	configuration [index_ + 5] = freeflyerPose_.getQuatRotation () [2];
	configuration [index_ + 6] = freeflyerPose_.getQuatRotation () [3];
      }

    protected:
      ExplicitRelativeTransformation
	(const std::string& name, const DevicePtr_t& robot,
	 const JointPtr_t& joint1, const JointPtr_t& joint2,
	 const Transform3f& frame1, const Transform3f& frame2,
	 std::vector <bool> mask = boost::assign::list_of (true)(true)(true)
	 (true)(true)(true)) :
	ExplicitNumericalConstraint (RelativeTransformation::create
				     (name, robot, joint1, joint2, frame1,
				      frame2, boost::assign::list_of (true)
				      (true)(true)(true)(true)(true)),
				     privOutputConf (joint2),
				     privOutputVelocity (joint2)),
	robot_ (robot), parentJoint_ (joint2->parentJoint ()->parentJoint ()),
	relativeTransformation_ (HPP_STATIC_PTR_CAST (RelativeTransformation,
						      functionPtr ())),
	index_ (joint2->parentJoint ()->rankInConfiguration ())
	{
	}

      ExplicitRelativeTransformation
	(const ExplicitRelativeTransformation& other) :
	ExplicitNumericalConstraint (other),
	robot_ (other.robot_), parentJoint_ (other.parentJoint_),
	relativeTransformation_ (other.relativeTransformation_),
	index_ (other.index_)
	{
	}

      // Store weak pointer to itself
      void init (const ExplicitRelativeTransformationWkPtr_t& weak)
	{
	  ExplicitNumericalConstraint::init (weak);
	  weak_ = weak;
	}
    private:
      SizeIntervals_t privOutputConf (const JointPtr_t& joint) {
	SizeIntervals_t result;
	result.push_back (SizeInterval_t
			  (joint->parentJoint ()->rankInConfiguration (), 7));
	return result;
      }
      SizeIntervals_t privOutputVelocity (const JointPtr_t& joint) {
	SizeIntervals_t result;
	result.push_back (SizeInterval_t
			  (joint->parentJoint ()->rankInVelocity (), 6));
	return result;
      }
      DevicePtr_t robot_;
      // Parent of the R3 joint.
      JointPtr_t parentJoint_;
      RelativeTransformationPtr_t relativeTransformation_;
      // Rank in configuration of the freeflyer joint
      size_type index_;
      mutable Transform3f freeflyerPose_;
      ExplicitRelativeTransformationWkPtr_t weak_;
    }; // class ExplicitRelativeTransformation
    /// \}
  } // namespace core
} // namespace hpp

#endif
