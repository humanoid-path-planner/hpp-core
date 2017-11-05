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

# include <hpp/constraints/matrix-view.hh>
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
    /// This class provides this expression. The input configuration variables
    /// are the joint values of all joints except the above mentioned freeflyer
    /// joint. The output configuration variables are the 7 configuration
    /// variables of the freeflyer joint.
    ///
    class HPP_CORE_DLLAPI ExplicitRelativeTransformation :
      public DifferentiableFunction
    {
    public:
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
	(const std::string& name      , const DevicePtr_t& robot,
	 const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
	 const Transform3f& frame1    , const Transform3f& frame2);

      ExplicitNumericalConstraintPtr_t createNumericalConstraint ()
      {
        return ExplicitNumericalConstraint::create (
            robot_,
            weak_.lock(),
            inConf_.indices(),
            inVel_.indices(),
            outConf_.indices(),
            outVel_.indices());
      }

    protected:
      typedef Eigen::BlockIndex BlockIndex;
      typedef Eigen::RowBlockIndices RowBlockIndices;
      typedef Eigen::ColBlockIndices ColBlockIndices;

      ExplicitRelativeTransformation (
          const std::string& name      , const DevicePtr_t& robot,
          const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
          const Transform3f& frame1    , const Transform3f& frame2,
          const segments_t inConf , const segments_t outConf,
          const segments_t inVel  , const segments_t outVel ,
          std::vector <bool> /*mask*/ = std::vector<bool>(6,true))
        : DifferentiableFunction (
              BlockIndex::cardinal(inConf),  BlockIndex::cardinal(inVel),
              pinocchio::LiegroupSpace::SE3 (), name),
          robot_ (robot),
          parentJoint_ (joint2->parentJoint ()),
          joint1_ (joint1), joint2_ (joint2),
          inConf_ (inConf),   inVel_  (inVel),
          outConf_ (outConf), outVel_ (outVel),
          F1inJ1_invF2inJ2_ (frame1 * frame2.inverse())
      {
      }

      ExplicitRelativeTransformation
	(const ExplicitRelativeTransformation& other) :
	DifferentiableFunction (other),
	robot_ (other.robot_),
        parentJoint_ (other.parentJoint_),
        inConf_ (other.inConf_),   inVel_  (other.inVel_),
        outConf_ (other.outConf_), outVel_ (other.outVel_),
        F1inJ1_invF2inJ2_ (other.F1inJ1_invF2inJ2_)
	{
	}

      // Store weak pointer to itself
      void init (const ExplicitRelativeTransformationWkPtr_t& weak)
      {
        weak_ = weak;
      }

      /// Compute the value (dimension 7) of the freeflyer joint 2
      ///
      /// \param argument vector of input configuration variables (all joints
      ///        except freeflyer joint)
      /// \retval result vector of output configuration variables corresponding
      ///         to the freeflyer value.
      void impl_compute (LiegroupElement& result, vectorIn_t argument) const;

      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const;

    private:
      void forwardKinematics (vectorIn_t arg) const;

      DevicePtr_t robot_;
      // Parent of the R3 joint.
      JointConstPtr_t parentJoint_;
      JointConstPtr_t joint1_, joint2_;
      RowBlockIndices inConf_;
      ColBlockIndices inVel_;
      RowBlockIndices outConf_ , outVel_;
      Transform3f F1inJ1_invF2inJ2_;

      ExplicitRelativeTransformationWkPtr_t weak_;

      // Tmp variables
      mutable vector_t qsmall_, q_;
      mutable Transform3f freeflyerPose_;
      mutable matrix3_t cross1_, cross2_;
      mutable matrix_t tmpJac_, J2_parent_minus_J1_;
    }; // class ExplicitRelativeTransformation
    /// \}
  } // namespace core
} // namespace hpp

#endif
