//
// Copyright (c) 2014,2015,2016, 2018 CNRS
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

#include <hpp/core/continuous-validation/solid-solid-collision.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision.h>

#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/core/deprecated.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {

      SolidSolidCollisionPtr_t SolidSolidCollision::create(const JointPtr_t& joint_a,
					      const ConstObjectStdVector_t& objects_b,
					      value_type tolerance)
      {
        SolidSolidCollision* ptr
          (new SolidSolidCollision(joint_a, objects_b, tolerance));
        SolidSolidCollisionPtr_t shPtr(ptr);
        ptr->init(shPtr);
        return shPtr;
      }

      SolidSolidCollisionPtr_t SolidSolidCollision::create(const JointPtr_t& joint_a,
					      const JointPtr_t& joint_b,
					      value_type tolerance)
      {
        SolidSolidCollision* ptr = new SolidSolidCollision
          (joint_a, joint_b, tolerance);
        SolidSolidCollisionPtr_t shPtr (ptr);
        ptr->init(shPtr);
        return shPtr;
      }

      SolidSolidCollisionPtr_t SolidSolidCollision::createCopy
      (const SolidSolidCollisionPtr_t& other)
      {
        SolidSolidCollision* ptr = new SolidSolidCollision (*other);
        SolidSolidCollisionPtr_t shPtr (ptr);
        ptr->init(shPtr);
        return shPtr;
      }
      IntervalValidationPtr_t SolidSolidCollision::copy () const
      {
        return createCopy(weak_.lock());
      }

      value_type SolidSolidCollision::computeMaximalVelocity(vector_t& Vb) const
      {
        value_type maximalVelocity_a_b = 0;
        for (const auto& coeff : m_->coefficients) {
            const JointPtr_t& joint = coeff.joint_;
            maximalVelocity_a_b += coeff.value_ * Vb.segment(joint->rankInVelocity(), joint->numberDof()).norm();
        }

        if(m_->joint_b)
        {
          value_type maximalVelocity_b_a = 0;
          for (const auto& coeff : m_->coefficients_reverse) {
            const JointPtr_t& joint = coeff.joint_;
            maximalVelocity_b_a += coeff.value_ * Vb.segment(joint->rankInVelocity(), joint->numberDof()).norm();
          }

          if (maximalVelocity_b_a < maximalVelocity_a_b)
          {
            return maximalVelocity_b_a;
          }
        }
        return maximalVelocity_a_b;
      }

      bool SolidSolidCollision::removeObjectTo_b (const CollisionObjectConstPtr_t& object)
      {
        CollisionPairs_t& prs (pairs());
        CollisionRequests_t& rqsts (requests());
        const int s = (int)prs.size();

        // Remove all reference to object
        int last = 0;
        for (int i = 0; i < s; ++i) {
          if (object != prs[i].second) { // Different -> keep
            if (last != i) { // If one has been removed, then move.
              prs[last] = std::move(prs[i]);
              rqsts[last] = std::move(rqsts[i]);
            }
            last++;
          }
        }
        prs.erase(prs.begin() + last, prs.end());
        rqsts.erase(rqsts.begin() + last, rqsts.end());

        return last != s;
      }

      void SolidSolidCollision::addCollisionPair (const CollisionObjectConstPtr_t& left,
          const CollisionObjectConstPtr_t& right)
      {
        // std::cout << "size = " << pairs().size() << std::endl;
        // std::cout << "capacity = " << pairs().capacity() << std::endl;
        pairs().emplace_back (left, right);
        requests().emplace_back (fcl::DISTANCE_LOWER_BOUND, 1);
        requests().back().enable_cached_gjk_guess = true;
      }

      std::string SolidSolidCollision::name () const
      {
        std::ostringstream oss;
        oss << "(" << m_->joint_a->name () << ",";
        if (m_->joint_b) oss << m_->joint_b->name ();
        else oss << "obstacles";
        oss << ")";
        return oss.str ();
      }

      std::ostream& SolidSolidCollision::print (std::ostream& os) const
      {
        const pinocchio::Model& model = joint_a()->robot ()->model();
        os << "SolidSolidCollision: " << m_->joint_a->name()
          << " - " << (m_->joint_b ? m_->joint_b->name() : model.names[0]) << '\n';
        JointIndices_t joints = m_->computeSequenceOfJoints ();
        for (auto i : joints)
          os << model.names[i] << ", ";
        os << '\n';
        for (std::size_t i = 0; i < m_->coefficients.size(); ++i)
          os << m_->coefficients[i].value_ << ", ";
        return os;
      }

      SolidSolidCollision::JointIndices_t SolidSolidCollision::Model::computeSequenceOfJoints () const
      {
        JointIndices_t joints;

        assert(joint_a);
        const pinocchio::Model& model = joint_a->robot ()->model();
        const JointIndex id_a = joint_a->index(),
                         id_b = (joint_b ? joint_b->index() : 0);
        JointIndex ia = id_a, ib = id_b;

        std::vector<JointIndex> fromA, fromB;
        while (ia != ib)
        {
          if (ia > ib) {
            fromA.push_back(ia);
            ia = model.parents[ia];
          } else /* if (ia < ib) */ {
            fromB.push_back(ib);
            ib = model.parents[ib];
          }
        }
        assert (ia == ib);
        fromA.push_back(ia);

        // Check joint vectors
        if (fromB.empty()) assert (fromA.back() == id_b);
        else               assert (model.parents[fromB.back()] == ia);

        // Build sequence
        joints = std::move(fromA);
        joints.insert(joints.end(), fromB.rbegin(), fromB.rend());
        assert(joints.front() == id_a);
        assert(joints.back() == id_b);
        assert(joints.size() > 1);
        return joints;
      }

      CoefficientVelocities_t SolidSolidCollision::Model::computeCoefficients(const JointIndices_t& joints) const
      {
        const pinocchio::Model& model = joint_a->robot ()->model();

        JointPtr_t child;
        assert (joints.size () > 1);
        CoefficientVelocities_t coeff;
        coeff.resize (joints.size () - 1);
        pinocchio::DevicePtr_t robot =joint_a->robot ();
        // Store r0 + sum of T_{i/i+1} in a variable
        value_type cumulativeLength = joint_a->linkedBody ()->radius ();
        value_type distance;
        std::size_t i = 0;
        while (i + 1 < joints.size()) {
          if (model.parents[joints[i]] == joints[i+1])
            child = Joint::create (robot, joints[i]);
          else if (model.parents[joints[i+1]] == joints[i])
            child = Joint::create (robot, joints[i+1]);
          else
            abort ();
          assert(child);
          coeff [i].joint_ = child;
          // Go through all known types of joints
          //  TODO: REPLACE THESE FUNCTIONS WITH NEW API
          distance = child->maximalDistanceToParent ();
          coeff [i].value_ =
            child->upperBoundLinearVelocity () +
            cumulativeLength * child->upperBoundAngularVelocity ();
          cumulativeLength += distance;

          ++i;
        }
        return coeff;
      }


      void SolidSolidCollision::Model::setCoefficients (const JointIndices_t& joints)
      {
        // Compute coefficients going from joint a to joint b
        coefficients = computeCoefficients (joints);

        // Compute coefficients going from joint b to joint a
        if(joint_b)
        {
          JointIndices_t joints_reverse(joints);
          std::reverse(joints_reverse.begin(),joints_reverse.end());
          coefficients_reverse = computeCoefficients (joints_reverse);
        }
      }

      SolidSolidCollision::SolidSolidCollision (const JointPtr_t& joint_a,
            const JointPtr_t& joint_b,
            value_type tolerance) :
        BodyPairCollision(tolerance), m_ (new Model)
      {
        m_->joint_a = joint_a;
        m_->joint_b = joint_b;

        assert (joint_a);
        if (joint_b && joint_b->robot () != joint_a->robot ()) {
          throw std::runtime_error
            ("Joints do not belong to the same device.");
        }
        if (indexJointA() == indexJointB()) {
          throw std::runtime_error ("Bodies should be different");
        }
        if (tolerance < 0) {
          throw std::runtime_error ("tolerance should be non-negative.");
        }

        if (joint_a) { assert(joint_a->linkedBody ()); }
        if (joint_b) { assert(joint_b->linkedBody ()); }
        // Find sequence of joints
        JointIndices_t joints (m_->computeSequenceOfJoints ());
        m_->setCoefficients (joints);
      }

      SolidSolidCollision::SolidSolidCollision (const JointPtr_t& joint_a,
            const ConstObjectStdVector_t& objects_b,
            value_type tolerance) :
        BodyPairCollision(tolerance), m_ (new Model)
      {
        m_->joint_a = joint_a;

        assert (joint_a);
        BodyPtr_t body_a = joint_a->linkedBody ();
        assert (body_a);
        for (size_type i = 0; i < body_a->nbInnerObjects(); ++i) {
	      CollisionObjectConstPtr_t obj = body_a->innerObjectAt(i);
          for (ConstObjectStdVector_t::const_iterator it = objects_b.begin ();
              it != objects_b.end (); ++it) {
            assert (!(*it)->joint () ||
                (*it)->joint ()->robot () != joint_a->robot ());
            addCollisionPair(obj, *it);
          }
        }
        // Find sequence of joints
        JointIndices_t joints (m_->computeSequenceOfJoints ());
        m_->setCoefficients (joints);
      }

      void SolidSolidCollision::init(const SolidSolidCollisionWkPtr_t& weak)
      {
        weak_ = weak;
      }

    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
