//
// Copyright (c) 2014,2015,2016,2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH
#define HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH

#include <hpp/core/continuous-validation/body-pair-collision.hh>

namespace hpp {
namespace core {
namespace continuousValidation {
struct CoefficientVelocity {
  CoefficientVelocity() : value_(0) {}
  /// Joint the degrees of freedom of which the bounds correspond to.
  JointPtr_t joint_;
  value_type value_;
};  // struct CoefficientVelocity
typedef std::vector<CoefficientVelocity> CoefficientVelocities_t;

/// Computation of collision-free sub-intervals of a path
///
/// This class derives from BodyPairCollision and validates a subinterval
/// of a path for collision between to robot bodies of between a robot
/// body and the environment. The robot body should be part of an open
/// kinematic chain.
///
/// See <a href="continuous-validation.pdf"> this document </a>
/// for details.
class HPP_CORE_DLLAPI SolidSolidCollision : public BodyPairCollision {
 public:
  /// Create instance and return shared pointer
  ///
  /// \param joint_a joint of the body to test for collision with the
  /// environment \param objects_b environment objects for the collision
  /// checking \param tolerance allowed penetration should be positive \pre
  /// objects_b should not be attached to a joint
  static SolidSolidCollisionPtr_t create(
      const JointPtr_t& joint_a, const ConstObjectStdVector_t& objects_b,
      value_type tolerance);

  /// Create instance and return shared pointer
  ///
  /// \param joint_a, joint_b joint of the bodies to test for collision
  /// \param tolerance allowed penetrationd be positive
  static SolidSolidCollisionPtr_t create(const JointPtr_t& joint_a,
                                         const JointPtr_t& joint_b,
                                         value_type tolerance);

  /// Copy instance and return shared pointer.
  static SolidSolidCollisionPtr_t createCopy(
      const SolidSolidCollisionPtr_t& other);

  value_type computeMaximalVelocity(vector_t& Vb) const;

  bool removeObjectTo_b(const CollisionObjectConstPtr_t& object);

  /// Set the break distance of each coal::CollisionRequest
  /// \sa void ContinuousValidation::breakDistance(value_type) const
  void breakDistance(value_type distance);

  std::string name() const;

  std::ostream& print(std::ostream& os) const;

  /// \note The left object should belong to joint_a and
  /// the right one should belong to joint_b, or vice-versa.
  /// This is not checked.
  void addCollisionPair(const CollisionObjectConstPtr_t& left,
                        const CollisionObjectConstPtr_t& right);

  // Get coefficients and joints
  const CoefficientVelocities_t& coefficients() const {
    return m_->coefficients;
  }

  /// Get joint a
  const JointPtr_t& joint_a() const { return m_->joint_a; }
  /// Get joint b
  const JointPtr_t& joint_b() const { return m_->joint_b; }

  /// Returns joint A index or -1 if no such joint exists.
  size_type indexJointA() const {
    return (m_->joint_a ? m_->joint_a->index() : 0);
  }
  /// Returns joint B index or -1 if no such joint exists.
  size_type indexJointB() const {
    return (m_->joint_b ? m_->joint_b->index() : 0);
  }

  IntervalValidationPtr_t copy() const;

 protected:
  /// Constructor of inter-body collision checking
  ///
  /// \param joint_a, joint_b joint of the bodies to test for collision
  /// \param tolerance allowed penetration should be positive
  /// \pre joint_a and joint_b should not be nul pointers.
  SolidSolidCollision(const JointPtr_t& joint_a, const JointPtr_t& joint_b,
                      value_type tolerance);

  /// Constructor of collision checking with the environment
  ///
  /// \param joint_a joint of the body to test for collision with the
  /// environment \param objects_b environment objects for the collision
  /// checking \param tolerance allowed penetration should be positive \pre
  /// objects_b should not be attached to a joint
  SolidSolidCollision(const JointPtr_t& joint_a,
                      const ConstObjectStdVector_t& objects_b,
                      value_type tolerance);

  /// Copy constructor
  SolidSolidCollision(const SolidSolidCollision& other)
      : BodyPairCollision(other), m_(other.m_) {}

  void init(const SolidSolidCollisionWkPtr_t& weak);

 private:
  typedef pinocchio::JointIndex JointIndex;
  typedef std::vector<JointIndex> JointIndices_t;

  struct Model {
    JointPtr_t joint_a;
    JointPtr_t joint_b;
    CoefficientVelocities_t coefficients;
    CoefficientVelocities_t coefficients_reverse;
    JointIndices_t computeSequenceOfJoints() const;
    CoefficientVelocities_t computeCoefficients(
        const JointIndices_t& joints) const;
    void setCoefficients(const JointIndices_t& joints);
  };
  shared_ptr<Model> m_;
  SolidSolidCollisionWkPtr_t weak_;
};  // class SolidSolidCollision
}  // namespace continuousValidation
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH
