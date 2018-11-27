//
// Copyright (c) 2014,2015,2016,2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH
# define HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH

#include <hpp/core/continuous-validation/body-pair-collision.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      struct CoefficientVelocity
      {
	      CoefficientVelocity () : value_ (0)
	      {
        }
        /// Joint the degrees of freedom of which the bounds correspond to.
        JointPtr_t joint_;
        value_type value_;
      }; // struct CoefficientVelocity
      typedef std::vector <CoefficientVelocity> CoefficientVelocities_t;

      /// Computation of collision-free sub-intervals of a path
      ///
      /// This class aims at validating a path for the absence of collision
      /// between two solid bodies of a robot.
      ///
      /// The interval of definition of the path is successively covered
      /// by intervals where boths bodies are proved to be collision-free.
      /// Each interval is computed by bounding from above the velocity of
      /// all points of body 1 in the reference frame of body 2.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details.
      class HPP_CORE_DLLAPI SolidSolidCollision : public BodyPairCollision
      {
      public:
        /// Create instance and return shared pointer
        ///
        /// \param joint_a joint of the body to test for collision with the environment
        /// \param objects_b environment objects for the collision checking
        /// \param tolerance allowed penetration should be positive
        /// \pre objects_b should not be attached to a joint
        static SolidSolidCollisionPtr_t create (const JointPtr_t& joint_a,
					      const ConstObjectStdVector_t& objects_b,
					      value_type tolerance);

        /// Create instance and return shared pointer
        ///
        /// \param joint_a, joint_b joint of the bodies to test for collision
        /// \param tolerance allowed penetrationd be positive
        static SolidSolidCollisionPtr_t create (const JointPtr_t& joint_a,
					      const JointPtr_t& joint_b,
					      value_type tolerance);

        value_type computeMaximalVelocity(vector_t& Vb) const;

        bool removeObjectTo_b (const CollisionObjectConstPtr_t& object);

	std::string name () const;

	std::ostream& print (std::ostream& os) const;

        /// \note The left object should belong to joint_a and
        /// the right one should belong to joint_b, or vice-versa.
        /// This is not checked.
        void addCollisionPair (const CollisionObjectConstPtr_t& left,
            const CollisionObjectConstPtr_t &right);

        // Get coefficients and joints
        const CoefficientVelocities_t& coefficients () const
        {
          return m_->coefficients;
        }

        /// Get joint a
        const JointPtr_t& joint_a () const
        {
          return m_->joint_a;
        }
        /// Get joint b
        const JointPtr_t& joint_b () const
        {
          return m_->joint_b;
        }

        /// Returns joint A index or -1 if no such joint exists.
        size_type indexJointA () const
        {
          return (m_->joint_a ? m_->joint_a->index() : 0);
        }
        /// Returns joint B index or -1 if no such joint exists.
        size_type indexJointB () const
        {
          return (m_->joint_b ? m_->joint_b->index() : 0);
        }

      protected:
        /// Constructor of inter-body collision checking
        ///
        /// \param joint_a, joint_b joint of the bodies to test for collision
        /// \param tolerance allowed penetration should be positive
        /// \pre joint_a and joint_b should not be nul pointers.
        SolidSolidCollision (const JointPtr_t& joint_a,
              const JointPtr_t& joint_b,
              value_type tolerance);

        /// Constructor of collision checking with the environment
        ///
        /// \param joint_a joint of the body to test for collision with the environment
        /// \param objects_b environment objects for the collision checking
        /// \param tolerance allowed penetration should be positive
        /// \pre objects_b should not be attached to a joint
        SolidSolidCollision (const JointPtr_t& joint_a,
              const ConstObjectStdVector_t& objects_b,
              value_type tolerance);

      private:
        typedef pinocchio::JointIndex JointIndex;
        typedef std::vector<JointIndex> JointIndices_t;

        struct Model {
          JointPtr_t joint_a;
          JointPtr_t joint_b;
          CoefficientVelocities_t coefficients;
          JointIndices_t computeSequenceOfJoints () const;
          void computeCoefficients (const JointIndices_t& joints);
        };
        boost::shared_ptr<Model> m_;
      }; // class SolidSolidCollision
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_SOLID_SOLID_COLLISION_HH
