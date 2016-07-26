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

#ifndef HPP_CORE_CONT_COLLISION_CHECKING_DICHOTOMY_BODY_PAIR_COLLISION_HH
# define HPP_CORE_CONT_COLLISION_CHECKING_DICHOTOMY_BODY_PAIR_COLLISION_HH

# include <limits>
# include <iterator>

# include <hpp/fcl/collision_data.h>
# include <hpp/fcl/collision.h>
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/collision-object.hh>
# include <hpp/pinocchio/joint.hh>
# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/projection-error.hh>
# include "continuous-collision-checking/intervals.hh"


namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      namespace dichotomy {
	HPP_PREDEF_CLASS (BodyPairCollision);
	typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;
	using pinocchio::JointPtr_t;
	using pinocchio::Transform3f;

	/// Multiplicative coefficients of linear and angular velocities
	struct CoefficientVelocity
	{
	  CoefficientVelocity () : value_ (0)
	  {
	  }
	  /// Joint the degrees of freedom of which the bounds correspond to.
	  JointConstPtr_t joint_;
	  value_type value_;
	}; // struct CoefficientVelocity

	/// Computation of collision-free sub-intervals of a path
	///
	/// This class aims at validating a path for the absence of collision
	/// between two bodies of a robot.
	///
	/// The interval of definition of the path is successively covered
	/// by intervals where boths bodies are proved to be collision-free.
	/// Each interval is computed by bounding from above the velocity of
	/// all points of body 1 in the reference frame of body 2.
	class BodyPairCollision
	{
	public:
	  /// Create instance and return shared pointer
	  ///
	  /// \param body_a body to test for collision with the environment
	  /// \param tolerance allowed penetration should be positive
	  /// \pre objects_b should not be attached to a joint
	  static BodyPairCollisionPtr_t create (const JointConstPtr_t& joint_a,
						const std::vector<CollisionObjectPtr_t>& objects_b,
						value_type tolerance)
	  {
	    BodyPairCollisionPtr_t shPtr (new BodyPairCollision
					  (joint_a, objects_b, tolerance));
	    return shPtr;
	  }

	  /// Create instance and return shared pointer
	  ///
	  /// \param body_a, body_b bodies to test for collision
	  /// \param tolerance allowed penetration should be positive
	  /// \pre body_a and body_b should not be nul pointers.
	  static BodyPairCollisionPtr_t create (const JointConstPtr_t& joint_a,
						const JointConstPtr_t& joint_b,
						value_type tolerance)
	  {
	    BodyPairCollisionPtr_t shPtr (new BodyPairCollision
					  (joint_a, joint_b, tolerance));
	    return shPtr;
	  }


	  const std::vector <JointConstPtr_t>& joints () const
	  {
	    return joints_;
	  }

	  /// Get joint a
	  const JointConstPtr_t& joint_a () const
	  {
	    return joint_a_;
	  }
	  /// Get joint b
	  const JointConstPtr_t& joint_b () const
	  {
	    return joint_b_;
	  }

	  void addObjectTo_b (const CollisionObjectPtr_t& object)
	  {
	    if (object->joint () &&
		object->joint ()->robot () == joint_a_->robot ()) {
	      throw std::runtime_error
		("Object should not be attached to a joint"
		 " to add it to a collision pair.");
	    }
	    objects_b_.push_back (object);
	  }

	  const std::vector<CollisionObjectPtr_t>& objects_b  () const
	  {
	    return objects_b_;
	  }

	  bool removeObjectTo_b (const CollisionObjectPtr_t& object)
	  {
	    for (std::vector<CollisionObjectPtr_t>::iterator itObj = objects_b_.begin ();
		 itObj != objects_b_.end (); ++itObj) {
	      if (object->fcl () == (*itObj)->fcl ()) {
		objects_b_.erase (itObj);
		return true;
	      }
	    }
	    return false;
	  }

	  /// Set path to validate
	  /// \param path path to validate,
	  /// Compute maximal velocity of point of body a in frame of body b
	  /// along the path.
	  void path (const StraightPathPtr_t& path)
	  {
	    path_ = path;
	    computeMaximalVelocity ();
	    intervals_.clear ();
	  }

	  /// Get path
	  PathConstPtr_t path () const
	  {
	    return path_;
	  }

	  /// Return the valid subset of the path for this collision pair.
	  const Intervals& validSubset () const
	  {
	    return intervals_;
	  }

	  /// Validate interval centered on a path parameter
	  /// \param t parameter value in the path interval of definition
	  /// \return true if the body pair is collision free for this parameter
	  ///         value, false if the body pair is in collision.
	  bool validateInterval
	  (const value_type& t, CollisionValidationReport& report)
	  {
	    using std::numeric_limits;
	    // Get configuration of robot corresponding to parameter
            bool success;
	    Configuration_t q = (*path_) (t, success);
            if (!success) throw
              projection_error(std::string ("Unable to apply constraints in ") + __PRETTY_FUNCTION__);
	    // Compute position of joint a in frame of common ancestor
	    pinocchio::Transform3f Ma, tmp;
        for (int i = (int)indexCommonAncestor_ - 1; i >= 0; --i) {
          joints_ [(std::size_t)i]->computePosition (q, Ma, tmp);
	      Ma = tmp;
	    }
	    // Compute position of joint b in frame of common ancestor
	    pinocchio::Transform3f Mb;
	    for (std::size_t i = indexCommonAncestor_ + 1; i < joints_.size ();
		 ++i) {
	      joints_ [i]->computePosition (q, Mb, tmp);
	      Mb = tmp;
	    }
	    value_type distanceLowerBound =
	      numeric_limits <value_type>::infinity ();
	    for (std::vector<CollisionObjectPtr_t>::const_iterator ita = objects_a_.begin ();
		 ita != objects_a_.end (); ++ita) {
	      // Compute position of object a
	      fcl::CollisionObject* object_a = (*ita)->fcl ().get ();
	      object_a->setTransform (Ma * (*ita)->positionInJointFrame ());
	      for (std::vector<CollisionObjectPtr_t>::const_iterator itb = objects_b_.begin ();
		   itb != objects_b_.end (); ++itb) {
		// Compute position of object b
		fcl::CollisionObject* object_b = (*itb)->fcl ().get ();
		object_b->setTransform (Mb * (*itb)->positionInJointFrame ());
		// Perform collision test
		fcl::CollisionRequest request (1, false, true, 1, false, true,
					       fcl::GST_INDEP);
		fcl::CollisionResult result;
		fcl::collide (object_a, object_b, request, result);
		// Get result
		if (result.isCollision ()) {
		  report.object1 = *ita;
		  report.object2 = *itb;
		  return false;
		}
		if (result.distance_lower_bound < distanceLowerBound) {
		  distanceLowerBound = result.distance_lower_bound;
		}
	      }
	    }
	    value_type halfLength;
	    if (distanceLowerBound ==
		numeric_limits <value_type>::infinity ()) {
	      halfLength = numeric_limits <value_type>::infinity ();
	    } else {
	      halfLength = (tolerance_ + distanceLowerBound)/maximalVelocity_;
	    }
	    std::string joint2;
	    if (joint_b_) joint2 = joint_b_->name ();
	    else joint2 = (*objects_b_.begin ())->name ();
	    assert (!isnan (halfLength));
	    intervals_.unionInterval
	      (interval_t(t - halfLength, t + halfLength));
	    return true;
	  }

	  value_type tolerance () const
	  {
	    return tolerance_;
	  }

	  value_type maximalVelocity () const
	  {
	    return maximalVelocity_;
	  }

	protected:
	  /// Constructor of inter-body collision checking
	  ///
	  /// \param body_a, body_b bodies to test for collision
	  /// \param tolerance allowed penetration should be positive
	  /// \pre body_a and body_b should not be nul pointers.
	  BodyPairCollision (const JointConstPtr_t& joint_a,
			     const JointConstPtr_t& joint_b,
			     value_type tolerance):
	    joint_a_ (joint_a), joint_b_ (joint_b), objects_a_ (),
	    objects_b_ (), joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	    tolerance_ (tolerance)
	  {
	    assert (joint_a);
	    assert (joint_b);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    BodyPtr_t body_b = joint_b_->linkedBody ();
	    assert (body_a);
	    assert (body_b);
	    objects_a_ = body_a->innerObjects ();
	    objects_b_ = body_b->innerObjects ();

	    if (joint_b_->robot () != joint_a_->robot ()) {
	      throw std::runtime_error
		("Joints do not belong to the same device.");
	    }
	    if (joint_a_ == joint_b_) {
	      throw std::runtime_error ("Bodies should be different");
	    }

	    if (tolerance < 0) {
	      throw std::runtime_error ("tolerance should be non-negative.");
	    }
	    //
	    // Find sequence of joints
	    computeSequenceOfJoints ();
	    computeCoefficients ();
	  }

	  /// Constructor of collision checking with the environment
	  ///
	  /// \param body_a body to test for collision with the environment
	  /// \param tolerance allowed penetration should be positive
	  /// \pre objects_b should not be attached to a joint
	  BodyPairCollision (const JointConstPtr_t& joint_a,
			     const std::vector<CollisionObjectPtr_t>& objects_b,
			     value_type tolerance) :
	    joint_a_ (joint_a), joint_b_ (), objects_a_ (), objects_b_ (),
	    joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	    tolerance_ (tolerance)
	  {
	    assert (joint_a);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    assert (body_a);
	    objects_a_ = body_a->innerObjects (pinocchio::COLLISION);
	    for (std::vector<CollisionObjectPtr_t>::const_iterator it = objects_b.begin ();
		 it != objects_b.end (); ++it) {
	      assert (!(*it)->joint () ||
		      (*it)->joint ()->robot () != joint_a_->robot ());
	    }
	    objects_b_ = objects_b;

	    if (tolerance < 0) {
	      throw std::runtime_error ("tolerance should be non-negative.");
	    }
	    //
	    // Find sequence of joints
	    computeSequenceOfJoints ();
	    computeCoefficients ();
	  }

	private:
	  void computeSequenceOfJoints ()
	  {
	    JointConstPtr_t j, jSave, commonAncestor = 0x0;
	    std::vector <JointConstPtr_t> ancestors1;
	    std::deque <JointConstPtr_t> ancestors2;
	    // Build vector of ancestors of joint farther away from common ancestor.
      pinocchio::Index minIdx, idx;
      if (joint_a_->index () > joint_b_->index ()) {
        idx = joint_a_->index ();
        minIdx = joint_a_->index ();
        j = joint_a_;
        jSave = joint_b_;
      } else {
        idx = joint_b_->index ();
        minIdx = joint_a_->index ();
        j = joint_b_;
        jSave = joint_a_;
      }
      while (idx != minIdx) {
        if (idx > minIdx) {
          //add jointIndex to list??
          ancestors1.push_back (j);
            // are inner objects the right choise here?
          j = j->robot ()->geomModel ()->innerObjects[idx].parent;
          idx = j->index ();
        }
      }
      commonAncestor = j;
      idx = minIxd;
      j = jSave;
      while (idx != commonAncestor->index ()) {
        if (idx > minIdx) {
          //add jointIndex to list??
          ancestors.push_back (j);
          // are inner objects the right choise here?
          j = j->robot ()->geomModel ()->innerObjects[idx].parent;
          idx = j->index ();
      }

      // after saving the two lists, find out which one is a ..?
      //


	    for (j = joint_a_; j; j = j->parentJoint ()) {
	      aAncestors.push_back (j);
	    }
	    aAncestors.push_back (0x0);
	    // Build vector of ancestors of joint_b in reverse order.
	    for (j = joint_b_; j; j = j->parentJoint ()) {
	      bAncestors.push_front (j);
	    }
	    // Find first common ancestor: can be none.
	    for (j1 = joint_a_; j1; j1 = j1 ? j1->parentJoint () : 0x0) {
	      for (j2 = joint_b_; j2; j2 = j2 ? j2->parentJoint () : 0x0) {
		if (j1 == j2) {
		  commonAncestor = j1;
		  j1 = JointConstPtr_t (new Joint ());
      j2 = JointConstPtr_t (new Joint ());
		}
	      }
	    }
	    // build sequence of joints
	    joints_.clear ();
	    for (std::vector <JointConstPtr_t>::const_iterator it =
		   aAncestors.begin ();
		 (it != aAncestors.end () && *it != commonAncestor); ++it) {
	      joints_.push_back (*it);
	    }
	    joints_.push_back (commonAncestor);
	    indexCommonAncestor_ = joints_.size () - 1;
	    bool commonAncestorFound = false;
	    for (std::deque <JointConstPtr_t>::const_iterator it =
		   bAncestors.begin (); it != bAncestors.end (); ++it) {
	      if (commonAncestorFound)
		joints_.push_back (*it);
	      else if (*it == commonAncestor)
		commonAncestorFound = true;
	    }
	  }

	  void computeCoefficients ()
	  {
	    JointConstPtr_t child;
	    assert (joints_.size () > 1);
	    coefficients_.resize (joints_.size () - 1);
	    // Store r0 + sum of T_{i/i+1} in a variable
	    value_type cumulativeLength = joint_a_->linkedBody ()->radius ();
	    value_type distance;
	    size_type i = 0;
	    std::vector <JointConstPtr_t>::const_iterator it = joints_.begin ();
	    std::vector <JointConstPtr_t>::const_iterator itNext = it + 1;
	    while (itNext != joints_.end ()) {
	      if ((*it)->parentJoint () == *itNext) {
		child = *it;
	      } else if ((*itNext)->parentJoint () == *it) {
		child = *itNext;
	      } else {
		abort ();
	      }
	      coefficients_ [i].joint_ = child;
	      // Go through all known types of joints
	      distance = child->maximalDistanceToParent ();
	      coefficients_ [i].value_ = child->upperBoundLinearVelocity () +
		cumulativeLength * child->upperBoundAngularVelocity ();
	      cumulativeLength += distance;
	      it = itNext; ++itNext; ++i;
	    }
	  }

	  /// Compute maximal velocity of points of body1 in the frame of body 2
	  /// \param path input path
	  void computeMaximalVelocity ()
	  {
	    value_type t0 = path_->timeRange ().first;
	    value_type t1 = path_->timeRange ().second;
	    value_type T = t1 - t0;
	    bool success;
	    Configuration_t q1 = (*path_) (t0, success);
	    Configuration_t q2 = (*path_) (t1, success);

	    maximalVelocity_ = 0;
	    for (std::vector <CoefficientVelocity>::const_iterator itCoef =
		   coefficients_.begin (); itCoef != coefficients_.end ();
		 ++itCoef) {
	      const JointConstPtr_t& joint = itCoef->joint_;
	      const value_type& value = itCoef->value_;
	      maximalVelocity_ += value * joint->configuration ()->distance
		(q1, q2, joint->rankInConfiguration ()) / T;
	    }
	  }

	  JointConstPtr_t joint_a_;
	  JointConstPtr_t joint_b_;
	  std::vector<CollisionObjectPtr_t> objects_a_;
    std::vector<CollisionObjectPtr_t> objects_b_;
	  std::vector <JointConstPtr_t> joints_;
	  std::size_t indexCommonAncestor_;
	  std::vector <CoefficientVelocity> coefficients_;
	  StraightPathPtr_t path_;
	  value_type maximalVelocity_;
	  Intervals intervals_;
	  value_type tolerance_;
	}; // class BodyPairCollision
      } // namespace dichotomy
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONT_COLLISION_CHECKING_DICHOTOMY_BODY_PAIR_COLLISION_HH
