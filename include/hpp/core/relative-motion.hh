//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_RELATIVE_MOTION_HH
#define HPP_CORE_RELATIVE_MOTION_HH

#include <Eigen/Core>
#include <hpp/core/fwd.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
namespace core {
struct RelativeMotion {
  enum RelativeMotionType {
    /// The relative motion is fully constrained and the constraint cannot be
    /// parameterized (has constant right-hand side)
    Constrained = 0,
    /// The relative motion is fully constrained but the constraint can be
    /// parameterized (has non-constant right-hand side)
    Parameterized = 1,
    /// The relative motion is not constrained
    Unconstrained = 2
  };

  /// Matrix of relative motion
  ///
  /// The row and column indices correspond to joint indices in the robot
  /// plus one. 0 corresponds to the environment.
  /// The values of the matrix are
  /// \li Constrained: the joints are rigidly fixed to each other,
  /// \li Parameterized: the joints are rigidly fixed to each other, but the
  /// relative transformation may differ from one path to another one,
  /// \li Unconstrained: the joints can move with respect to each other.
  typedef Eigen::Matrix<RelativeMotionType, Eigen::Dynamic, Eigen::Dynamic>
      matrix_type;

  /// Build a new RelativeMotion matrix from a robot
  ///
  /// \param robot a Device,
  /// initialize a matrix of size (N+1) x (N+1) where N is the robot number
  /// of degrees of freedom.
  /// Diagonal elements are set to RelativeMotion::Constrained,
  /// other elements are set to RelativeMotion::Unconstrained.
  static matrix_type matrix(const DevicePtr_t& robot);

  /// Fill the relative motion matrix with information extracted from the
  /// provided ConstraintSet.
  /// \note Only LockedJoint and RelativeTransformation of dimension 6
  ///       are currently taken into account.
  /// \todo LockedJoint always has a non-constant RHS which means it will
  ///       always be treated a parameterized constraint. Even when the
  ///       value is not going to change...
  static void fromConstraint(matrix_type& matrix, const DevicePtr_t& robot,
                             const ConstraintSetPtr_t& constraint);

  /// Set the relative motion between two joints
  ///
  /// This does nothing if type is Unconstrained.
  /// The full matrix is updated as follow. For any indices i0 and i3
  /// different from both i1 and i2:
  /// - set matrix(i0,i2) if i0 and i1 was constrained,
  /// - set matrix(i1,i3) if i2 and i3 was constrained,
  /// - set matrix(i0,i3) if the two previous condition are fulfilled.
  ///
  /// The RelativeMotionType is deduced as follow:
  /// - RelativeMotion::Constrained   + RelativeMotion::Parameterized ->
  /// RelativeMotion::Parameterized
  /// - RelativeMotion::Constrained   + RelativeMotion::Constrained   ->
  /// RelativeMotion::Constrained
  /// - RelativeMotion::Parameterized + RelativeMotion::Parameterized ->
  /// RelativeMotion::Parameterized
  /// - RelativeMotion::Unconstrained +                 *             ->
  /// RelativeMotion::Unconstrained
  static void recurseSetRelMotion(matrix_type& matrix, const size_type& i1,
                                  const size_type& i2,
                                  const RelativeMotionType& type);

  /// Get the index for a given joint
  ///
  /// \return 0 if joint is NULL, joint->index() otherwise.
  static inline size_type idx(const JointConstPtr_t& joint) {
    return (joint ? joint->index() : 0);
  }
};
}  // namespace core
}  // namespace hpp

namespace Eigen {
template <>
struct NumTraits<hpp::core::RelativeMotion::RelativeMotionType>
    : NumTraits<int> {};
}  // namespace Eigen

#endif  // HPP_CORE_RELATIVE_MOTION_HH
