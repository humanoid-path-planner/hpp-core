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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH

# include <limits>
# include <iterator>

# include <fcl/collision_data.h>
# include <fcl/collision.h>
# include <hpp/model/body.hh>
# include <hpp/model/collision-object.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/straight-path.hh>
# include "continuous-collision-checking/intervals.hh"


namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      using model::JointPtr_t;
      using model::JointTranslation;
      using model::JointRotation;
      using model::JointSO3;
      using model::JointAnchor;
      using model::JointTranslationPtr_t;
      using model::JointRotationPtr_t;
      using model::JointSO3Ptr_t;
      using model::JointAnchorPtr_t;
      using model::JointTranslationConstPtr_t;
      using model::JointRotationConstPtr_t;
      using model::JointSO3ConstPtr_t;
      using model::JointAnchorConstPtr_t;
      using model::Transform3f;

      struct Object
      {
	Object (const CollisionObjectPtr_t& collisionObject) :
	  fcl_ (collisionObject->fcl ()),
	  positionInJointFrame_ (collisionObject->positionInJointFrame ()),
	  name_ (collisionObject->name ())
	{
	}
	fcl::CollisionObjectPtr_t fcl_;
	Transform3f positionInJointFrame_;
	std::string name_;
      }; // struct Object

      typedef std::vector <Object> Objects_t;

      Objects_t store (const ObjectVector_t& collisionObjects)
      {
	// Pre-allocate memory
	Objects_t result;
	result.reserve (collisionObjects.size ());
	for (ObjectVector_t::const_iterator itObj = collisionObjects.begin ();
	     itObj != collisionObjects.end (); ++itObj) {
	  result.push_back (Object (*itObj));
	}
	return result;
      }
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
					      const ObjectVector_t& objects_b,
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
	    throw std::runtime_error ("Object should not be attached to a joint"
				      " to add it to a collision pair.");
	  }
	  objects_b_.push_back (object);
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

	/// Return the valid subset of the path for this collision pair.
	const Intervals& validSubset () const
	{
	  return intervals_;
	}

	/// Validate interval centered on a path parameter
	/// \param t parameter value in the path interval of definition
	/// \return true if the body pair is collision free for this parameter
	///         value, false if the body pair is in collision.
	bool validateInterval (const value_type& t)
	{
	  using std::numeric_limits;
	  // Get configuration of robot corresponding to parameter
	  Configuration_t q = (*path_) (t);
	  // Compute position of joint a in frame of common ancestor
	  model::Transform3f Ma;
	  for (int i = indexCommonAncestor_ - 1; i >= 0; --i) {
	    joints_ [(std::size_t)i]->computePosition (q, Ma, Ma);
	  }
	  // Compute position of joint b in frame of common ancestor
	  model::Transform3f Mb;
	  for (std::size_t i = indexCommonAncestor_ + 1; i < joints_.size ();
	       ++i) {
	    joints_ [i]->computePosition (q, Mb, Mb);
	  }
	  value_type distanceLowerBound =
	    numeric_limits <value_type>::infinity ();
	  for (Objects_t::const_iterator ita = objects_a_.begin ();
	       ita != objects_a_.end (); ++ita) {
	    // Compute position of object a
	    fcl::CollisionObject* object_a = ita->fcl_.get ();
	    object_a->setTransform (Ma * ita->positionInJointFrame_);
	    for (Objects_t::const_iterator itb = objects_b_.begin ();
	       itb != objects_b_.end (); ++itb) {
	      // Compute position of object b
	      fcl::CollisionObject* object_b = itb->fcl_.get ();
	      object_b->setTransform (Mb * itb->positionInJointFrame_);
	      // Perform collision test
	      fcl::CollisionRequest request (1, false, true, 1, false, true,
					     fcl::GST_INDEP);
	      fcl::CollisionResult result;
	      fcl::collide (object_a, object_b, request, result);
	      // Get result
	      if (result.isCollision ()) return false;
	      if (result.distance_lower_bound < distanceLowerBound) {
		distanceLowerBound = result.distance_lower_bound;
	      }
	    }
	  }
	  value_type halfLength;
	  if (distanceLowerBound == numeric_limits <value_type>::infinity ()) {
	    halfLength = numeric_limits <value_type>::infinity ();
	  } else {
	    halfLength = (tolerance_ + distanceLowerBound)/maximalVelocity_;
	  }
	  assert (!isnan (halfLength));
	  intervals_.unionInterval (interval_t(t - halfLength, t + halfLength));
	  return true;
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
	  joint_a_ (joint_a), joint_b_ (joint_b), objects_a_ (), objects_b_ (),
	  joints_ (),
	  indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	  tolerance_ (tolerance)
	{
	  assert (joint_a);
	  assert (joint_b);
	  BodyPtr_t body_a = joint_a_->linkedBody ();
	  BodyPtr_t body_b = joint_b_->linkedBody ();
	  assert (body_a);
	  assert (body_b);
	  objects_a_ = store (body_a->innerObjects (model::COLLISION));
	  objects_b_ = store (body_b->innerObjects (model::COLLISION));

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
			   const ObjectVector_t& objects_b,
			   value_type tolerance) :
	  joint_a_ (joint_a), joint_b_ (), objects_a_ (), objects_b_ (),
	  joints_ (),
	  indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	  tolerance_ (tolerance)
	{
	  assert (joint_a);
	  BodyPtr_t body_a = joint_a_->linkedBody ();
	  assert (body_a);
	  objects_a_ = store (body_a->innerObjects (model::COLLISION));
	  for (ObjectVector_t::const_iterator it = objects_b.begin ();
	       it != objects_b.end (); ++it) {
	    assert (!(*it)->joint () ||
		    (*it)->joint ()->robot () != joint_a_->robot ());
	  }
	  objects_b_ = store (objects_b);

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
	  JointConstPtr_t j1, j2, j, commonAncestor = 0x0;
	  std::vector <JointConstPtr_t> aAncestors;
	  std::deque <JointConstPtr_t> bAncestors;
	  // Build vector of ancestors of joint_a.
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
		j1 = 0x0; j2 = 0x0;
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
	    const Transform3f& pos = child->positionInParentFrame ();
	    fcl::Vec3f T = pos.getTranslation ();
	    if (JointTranslationConstPtr_t joint =
		dynamic_cast <JointTranslationConstPtr_t> (child)) {
	      fcl::Vec3f u = pos.getRotation ().getColumn (0);
	      if (joint->isBounded (0)) {
		distance = std::max ((T + joint->lowerBound (0)*u).length (),
				     (T + joint->upperBound (0)*u).length ());
		coefficients_ [i].value_ = 1.;
	      }	else {
		distance = std::numeric_limits <value_type>::infinity ();
	      }
	    } else if (dynamic_cast <JointRotationConstPtr_t> (child)) {
	      distance = T.length ();
	      coefficients_ [i].value_ = cumulativeLength;
	    } else if (dynamic_cast <JointSO3ConstPtr_t> (child)) {
	      distance = T.length ();
	      coefficients_ [i].value_ = cumulativeLength;
	    } else if (dynamic_cast <JointAnchorConstPtr_t> (child)) {
	      distance = T.length ();
	      coefficients_ [i].value_ = 0;
	    } else {
	      throw std::runtime_error ("Unknown type of joint.");
	    }
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
	  Configuration_t q1 = (*path_) (t0);
	  Configuration_t q2 = (*path_) (t1);

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
	Objects_t objects_a_;
	Objects_t objects_b_;
	std::vector <JointConstPtr_t> joints_;
	std::size_t indexCommonAncestor_;
	std::vector <CoefficientVelocity> coefficients_;
	StraightPathPtr_t path_;
	value_type maximalVelocity_;
	Intervals intervals_;
	value_type tolerance_;
      }; // class BodyPairCollision
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH
