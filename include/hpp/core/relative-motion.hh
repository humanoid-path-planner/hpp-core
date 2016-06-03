//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_RELATIVE_MOTION_HH
#define HPP_CORE_RELATIVE_MOTION_HH

#include <Eigen/Core>

#include <hpp/model/fwd.hh>
#include <hpp/core/fwd.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace core {
    struct RelativeMotion {
      enum RelativeMotionType {
        /// The relative motion is fully constrained and the constraint cannot be
        /// parameterized (has constant right-hand side)
        Constrained,
        /// The relative motion is fully constrained but the constraint can be
        /// parameterized (has non-constant right-hand side)
        Parameterized,
        /// The relative motion is not constrained
        Unconstrained
      };

      typedef Eigen::Matrix<RelativeMotionType, Eigen::Dynamic, Eigen::Dynamic> matrix_type;

      /// Build a new RelativeMotion matrix from a robot
      ///
      /// \param robot a Device,
      /// initialize a matirx of size (N+1) x (N+1) where N is the robot number
      /// of degrees of freedom.
      /// Diagonal elements are set to RelativeMotion::Constrained,
      /// other elements are set to RelativeMotion::Unconstrained.
      static matrix_type matrix (const DevicePtr_t& robot);

      /// Fill the relative motion matrix with information extracted from the
      /// provided ConstraintSet.
      /// \note Only LockedJoint and RelativeTransformation of dimension 6
      ///       are currently taken into account.
      /// \todo LockedJoint always has a non-constant RHS which means it will
      ///       always be treated a parameterized constraint. Even when the
      ///       value is not going to change...
      static void fromConstraint (
          matrix_type& matrix,
          const DevicePtr_t& robot,
          const ConstraintSetPtr_t& constraint);

      /// Set the relative motion between two joints
      ///
      /// This does nothing if type is Unconstrained.
      /// The full matrix is updated as follow. For any indexes i0 and i3
      /// different from both i1 and i2:
      /// - set matrix(i0,i2) if i0 and i1 was constrained,
      /// - set matrix(i1,i3) if i2 and i3 was constrained,
      /// - set matrix(i0,i3) if the two previous condition are fulfilled.
      ///
      /// The RelativeMotionType is deduced as follow:
      /// - RelativeMotion::Constrained   + RelativeMotion::Parameterized -> RelativeMotion::Parameterized
      /// - RelativeMotion::Constrained   + RelativeMotion::Constrained   -> RelativeMotion::Constrained
      /// - RelativeMotion::Parameterized + RelativeMotion::Parameterized -> RelativeMotion::Parameterized
      /// - RelativeMotion::Unconstrained +                 *             -> RelativeMotion::Unconstrained
      static void recurseSetRelMotion(matrix_type& matrix,
          const size_type& i1, const size_type& i2,
          const RelativeMotionType& type);

      /// Get the index for a given joint
      ///
      /// \return 0 if joint is NULL, joint->rankInVelocity()+1 otherwise.
      static size_type idx(const JointPtr_t& joint);
    };
  } // namespace core
} // namespace hpp

namespace Eigen {
  template<> struct NumTraits<hpp::core::RelativeMotion::RelativeMotionType>
    : NumTraits<int> {};
} // namespace Eigen

#endif // HPP_CORE_RELATIVE_MOTION_HH
