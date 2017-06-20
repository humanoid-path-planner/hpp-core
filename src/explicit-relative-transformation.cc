// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/core/explicit-relative-transformation.hh>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/skew.hpp>

#include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace core {
    namespace {
      typedef JointJacobian_t::ConstNRowsBlockXpr<3>::Type ConstHalfJacobian_t;
      inline ConstHalfJacobian_t omega(const JointJacobian_t& j) { return j.bottomRows<3>(); }
      inline ConstHalfJacobian_t trans(const JointJacobian_t& j) { return j.topRows<3>(); }

      void inputVariable (JointConstPtr_t joint, std::vector<bool>& conf, std::vector<bool>& vel)
      {
        while (joint && joint->index() != 0) {
          for (size_type i = 0; i < joint->configSize(); ++i)
            conf[joint->rankInConfiguration() + i] = !conf[joint->rankInConfiguration() + i];
          for (size_type i = 0; i < joint->numberDof(); ++i)
            vel[joint->rankInVelocity() + i] = !vel[joint->rankInVelocity() + i];
          joint = joint->parentJoint();
        }
      }

      typedef Eigen::BlockIndex<size_type> BlockIndex;

      BlockIndex::vector_t vectorOfBoolToIntervals (std::vector<bool>& v)
      {
        BlockIndex::vector_t ret;
        for (std::size_t i = 0; i < v.size(); ++i)
          if (v[i]) ret.push_back(BlockIndex::type(i, 1));
        BlockIndex::shrink (ret);
        return ret;
      }

      BlockIndex::vector_t jointConfInterval (JointConstPtr_t j) {
        return BlockIndex::vector_t(1, BlockIndex::type(j->rankInConfiguration(), j->configSize()));
      }
      BlockIndex::vector_t jointVelInterval (JointConstPtr_t j) {
        return BlockIndex::vector_t(1, BlockIndex::type(j->rankInVelocity(), j->numberDof()));
      }
    }

    ExplicitRelativeTransformationPtr_t ExplicitRelativeTransformation::create
      (const std::string& name      , const DevicePtr_t& robot,
       const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
       const Transform3f& frame1    , const Transform3f& frame2)
      {
        std::vector<bool> conf (robot->configSize(), false);
        std::vector<bool> vel  (robot->numberDof(), false);
        inputVariable (joint1, conf, vel);
        inputVariable (joint2->parentJoint(), conf, vel);

        ExplicitRelativeTransformation* ptr =
          new ExplicitRelativeTransformation (name, robot, joint1, joint2,
              frame1, frame2,
              vectorOfBoolToIntervals(conf),
              jointConfInterval(joint2),
              vectorOfBoolToIntervals(vel),
              jointVelInterval(joint2));
        ExplicitRelativeTransformationPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

    void ExplicitRelativeTransformation::forwardKinematics (vectorIn_t arg) const
    {
      qsmall_ = inConf_.rview(robot_->currentConfiguration());
      if (qsmall_ != arg) {
        q_ = robot_->currentConfiguration();
        inConf_.lview(q_) = arg;
        robot_->currentConfiguration(q_);
      }
      robot_->computeForwardKinematics ();
    }

    void ExplicitRelativeTransformation::impl_compute (vectorOut_t result, vectorIn_t argument) const
    {
      forwardKinematics (argument);

      // J1 * M1/J1 = J2 * M2/J2
      // J2 = J1 * M1/J1 * M2/J2^{-1}
      // J2 = J2_{parent} * T
      // T = J2_{parent}^{-1} * J2
      // T = J2_{parent}^{-1} * J1 * F1/J1 * F2/J2^{-1}
      freeflyerPose_ =
        implicit_->joint1 ()->currentTransformation () *
        F1inJ1_invF2inJ2_;

      if (parentJoint_)
        freeflyerPose_ = parentJoint_->currentTransformation ().actInv(freeflyerPose_);

      freeflyerPose_ =
        implicit_->joint2 ()->positionInParentFrame ().actInv (freeflyerPose_);

      typedef Transform3f::Quaternion_t Q_t;
      result.head<3>() = freeflyerPose_.translation();
      result.tail<4>() = Q_t(freeflyerPose_.rotation()).coeffs();
    }

    void ExplicitRelativeTransformation::impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const
    {
      forwardKinematics (arg);
#ifndef NDEBUG
      // That the function has already been solved.
      vector_t result (7);
      impl_compute (result, arg);
      assert (outConf_.rview(robot_->currentConfiguration()).eval().isApprox(result));
#endif

      const JointJacobian_t& J1 (implicit_->joint1()->jacobian());
      // const JointJacobian_t& J2_parent (parentJoint_->jacobian());

      const matrix3_t& R1        (implicit_->joint1()->currentTransformation().rotation());
      const matrix3_t& R2        (implicit_->joint2()->currentTransformation().rotation());
      // const matrix3_t& R2_parent (parentJoint_       ->currentTransformation().rotation());
      const matrix3_t& R2_inParentFrame (implicit_->joint2()->positionInParentFrame().rotation());

      const vector3_t& t1        (implicit_->joint1()->currentTransformation().translation());

      cross1_ = se3::skew((R1 * F1inJ1_invF2inJ2_.translation()).eval());
      if (parentJoint_) {
        const vector3_t& t2_parent (parentJoint_       ->currentTransformation().translation());
        cross2_ = se3::skew((t2_parent - t1).eval());

        J2_parent_minus_J1_.noalias() = parentJoint_->jacobian() - J1;
      } else {
        cross2_ = - se3::skew(t1);
        // J2_parent_minus_J1_ = - J1;
      }

      // Express velocity of J1 * M1/J1 * M2/J2^{-1} in J2_{parent}.
      if (parentJoint_) {
        const matrix3_t&       R2_parent (parentJoint_->currentTransformation().rotation());
        const JointJacobian_t& J2_parent (parentJoint_->jacobian());

        tmpJac_.noalias() = (R2_inParentFrame.transpose() * R2_parent.transpose()) *
          ( cross1_ * (omega(J2_parent_minus_J1_))
            - cross2_ * omega(J2_parent)
            - trans(J2_parent_minus_J1_));
      } else {
        tmpJac_.noalias() = R2.transpose() *
          ( (- cross1_ * R1) * omega(J1) + R1 * trans(J1));
      }

      jacobian.topRows<3>() = inVel_.rview(tmpJac_);
      // jacobian.topRows<3>().setZero();

      if (parentJoint_) {
        const matrix3_t&       R2_parent (parentJoint_->currentTransformation().rotation());
        const JointJacobian_t& J2_parent (parentJoint_->jacobian());

        // J = p2RT2 * 0RTp2 * [ p2
        tmpJac_.noalias() = ( R2.transpose() * R2_parent ) * omega(J2_parent)
          - (R2.transpose() * R1) * omega(J1);
      } else {
        tmpJac_.noalias() = ( R2.transpose() * R1 ) * omega(J1);
      }

      jacobian.bottomRows<3>() = inVel_.rview(tmpJac_);
      // jacobian.bottomRows<3>().setZero();

      /*
      hppDout (info, "Computed jacobian " << jacobian);

      matrix_t other (6, rt_->inputDerivativeSize());
      rt_->jacobian (other, robot_->currentConfiguration());

      hppDout (info, "Other jacobian " << other);
      hppDout (info, "Other jacobian viewed " << inVel_.rview(other).eval());
      hppDout (info, "Other jacobian - jacobian " << inVel_.rview(other).eval() - jacobian);
      */

      // jacobian = inVel_.rview(other);
    }
  } // namespace core
} // namespace hpp
