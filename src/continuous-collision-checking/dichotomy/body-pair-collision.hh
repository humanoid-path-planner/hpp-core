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
#include <pinocchio/multibody/geometry.hpp>
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
	  JointPtr_t joint_;
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
	  static BodyPairCollisionPtr_t create (const JointPtr_t& joint_a,
                                                const ConstObjectStdVector_t& objects_b,
						value_type tolerance)
    {
        std::vector<CollisionObjectConstPtr_t> obs;
        for (unsigned int i = 0; i<objects_b.size (); ++i) { 
            obs.push_back(boost::const_pointer_cast<const pinocchio::CollisionObject>(objects_b[i]));
        }
	    BodyPairCollisionPtr_t shPtr (new BodyPairCollision
					  (joint_a, obs, tolerance));
	    return shPtr;
	  }

	  /// Create instance and return shared pointer
	  ///
	  /// \param body_a, body_b bodies to test for collision
	  /// \param tolerance allowed penetration should be positive
	  /// \pre body_a and body_b should not be nul pointers.
	  static BodyPairCollisionPtr_t create (const JointPtr_t& joint_a,
						const JointPtr_t& joint_b,
						value_type tolerance)
	  {
      // TODO: FIX the const / non-const mixup! Conversion only as a quick fix
      //JointPtr_t ja = boost::const_pointer_cast<const pinocchio::Joint>(joint_a);
      //JointPtr_t jb = boost::const_pointer_cast<const pinocchio::Joint>(joint_b);
	    BodyPairCollisionPtr_t shPtr (new BodyPairCollision
					  (joint_a, joint_b, tolerance));
	    return shPtr;
	  }


	  const std::vector <JointPtr_t>& joints () const
	  {
	    return joints_;
	  }

	  /// Get joint a
	  const JointPtr_t& joint_a () const
	  {
	    return joint_a_;
	  }
	  /// Get joint b
	  const JointPtr_t& joint_b () const
	  {
	    return joint_b_;
	  }

	  void addObjectTo_b (const CollisionObjectConstPtr_t& object)
	  {
	    if (object->joint () &&
		object->joint ()->robot () == joint_a_->robot ()) {
	      throw std::runtime_error
		("Object should not be attached to a joint"
		 " to add it to a collision pair.");
	    }
	    objects_b_.push_back (object);
	  }

	  const std::vector<CollisionObjectConstPtr_t>& objects_b  () const
	  {
	    return objects_b_;
	  }

	  bool removeObjectTo_b (const CollisionObjectConstPtr_t& object)
	  {
	    for (std::vector<CollisionObjectConstPtr_t>::iterator itObj = objects_b_.begin ();
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
      pinocchio::DevicePtr_t robot =boost::const_pointer_cast<pinocchio::Device>(joint_a_->robot ());
	    using std::numeric_limits;
	    // Get configuration of robot corresponding to parameter
            bool success;
            const pinocchio::Configuration_t q = (*path_) (t, success);
            if (!success) throw
              projection_error(std::string ("Unable to apply constraints in ") + __PRETTY_FUNCTION__);
	    // Compute position of joint a in frame of common ancestor
	    pinocchio::Transform3f Ma;
      Ma.setIdentity ();
      const pinocchio::Configuration_t& qSave = robot->currentConfiguration ();
      robot->currentConfiguration (q);
      robot->computeForwardKinematics ();
        for (int i = (int)indexCommonAncestor_ - 1; i >= 0; --i) {
          
          // Old API:
          // joints_ [(std::size_t)i]->computePosition (q, Ma, tmp);
          // Would this work..? :
          // idx = joints_ [(std::size_t)i]->index ();
          // jData = robot->data ().joints[idx];
          // robot->model ().joints[idx].calc (jData, q);
          // Ma = jData.M ();
          // Current solution:
	        Ma = joints_ [(std::size_t)i]->currentTransformation ();
	    }
	    // Compute position of joint b in frame of common ancestor
	    pinocchio::Transform3f Mb;
      Mb.setIdentity ();
	    for (std::size_t i = indexCommonAncestor_ + 1; i < joints_.size (); ++i) {
	      // joints_ [i]->computePosition (q, Mb, tmp);
	      Ma = joints_ [(std::size_t)i]->currentTransformation ();
	    }
	    value_type distanceLowerBound =
	      numeric_limits <value_type>::infinity ();
	    for (std::vector<CollisionObjectConstPtr_t>::const_iterator ita = objects_a_.begin ();
		 ita != objects_a_.end (); ++ita) {
	      // Compute position of object a
        pinocchio::FclCollisionObjectPtr_t object_a =
        const_cast<fcl::CollisionObject*> ((*ita)->fcl ());
	      object_a->setTransform (se3::toFclTransform3f(Ma * (*ita)->positionInJointFrame ()));
	      for (std::vector<CollisionObjectConstPtr_t>::const_iterator itb = objects_b_.begin ();
		            itb != objects_b_.end (); ++itb) {
		      // Compute position of object b
          pinocchio::FclCollisionObjectPtr_t object_b =
            const_cast<fcl::CollisionObject*> ((*itb)->fcl ());
		      object_b->setTransform (se3::toFclTransform3f(Mb * (*itb)->positionInJointFrame ()));
		      // Perform collision test
		      fcl::CollisionRequest request (1, false, true, 1,
                false, true, fcl::GST_INDEP);
		      fcl::CollisionResult result;
		      fcl::collide (object_a, object_b, request, result);
          // TODO: where should the configuration be set back to original?
          robot->currentConfiguration (qSave);
		      // Get result
		      if (result.isCollision ()) {
		        report.object1 = boost::const_pointer_cast<pinocchio::CollisionObject>(*ita);
		        report.object2 = boost::const_pointer_cast<pinocchio::CollisionObject>(*itb);
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
	  BodyPairCollision (const JointPtr_t& joint_a,
			     const JointPtr_t& joint_b,
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
      // TODO:: optimise!!
      for (std::size_t i = 0; i < body_a->innerObjects ().size (); ++i) {
	    objects_a_.push_back (body_a->innerObjects ().at(i));
      }
      for (std::size_t i = 0; i < body_b->innerObjects ().size (); ++i) {
	    objects_b_.push_back (body_b->innerObjects ().at(i));
      }

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
	  BodyPairCollision (const JointPtr_t& joint_a,
			     const std::vector<CollisionObjectConstPtr_t>& objects_b,
			     value_type tolerance) :
	    joint_a_ (joint_a), joint_b_ (), objects_a_ (), objects_b_ (),
	    joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	    tolerance_ (tolerance)
	  {
	    assert (joint_a);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    assert (body_a);
      for (unsigned int i = 0; i < body_a->innerObjects ().size (); ++i) {
	        objects_a_.push_back (body_a->innerObjects ().at(i));
      }
	    for (std::vector<CollisionObjectConstPtr_t>::const_iterator it = objects_b.begin ();
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
	    JointPtr_t j, commonAncestor;
	    std::vector <JointPtr_t> aAncestors;
      aAncestors.clear ();
	    std::deque <JointPtr_t> bAncestors;
      bAncestors.clear ();
      pinocchio::Joint::Index minIdx, idx;
      pinocchio::DeviceConstPtr_t robot =joint_a_->robot ();

      if (joint_a_->index () > joint_b_->index ()) {
        idx = joint_a_->index ();
        minIdx = joint_a_->index ();
        j = joint_a_;
      } else {
        idx = joint_b_->index ();
        minIdx = joint_a_->index ();
        j = joint_b_;
      }

      while (idx > minIdx) {
        idx = robot->model ().parents[idx];
      }

      // TODO:: Check for possibility of not having a common ancestor???
      commonAncestor = pinocchio::JointPtr_t (new pinocchio::Joint 
              (boost::const_pointer_cast<pinocchio::Device>(robot), idx));

      idx = joint_a_->index ();
      j = joint_a_;
      while (idx != commonAncestor->index ()) {
          // only add to ancestors of joint_a_, not the joint itself
          if (idx != joint_a_->index ()) {
	          aAncestors.push_back (j);
          }
          idx = robot->model ().parents[idx];
          j = pinocchio::JointPtr_t (new pinocchio::Joint 
              (boost::const_pointer_cast<pinocchio::Device>(robot), idx));
	    } // if joint_a_ is the root joint, it will have no ancestors
        // and the vector is empty

      idx = joint_b_->index ();
      j = joint_b_;
	    // Build vector of ancestors of joint_b in reverse order.
	    while (idx != commonAncestor->index ()) {
          if (idx != joint_b_->index ()) {
	          bAncestors.push_front (j);
          }
          idx = robot->model ().parents[idx];
          j = pinocchio::JointPtr_t (new pinocchio::Joint 
              (boost::const_pointer_cast<pinocchio::Device>(robot), idx));
      }

	    // build sequence of joints
	    joints_.clear ();
      joints_.push_back (joint_a_);
	    for (std::vector <JointPtr_t>::const_iterator it =
		   aAncestors.begin (); it != aAncestors.end (); ++it) {
	      joints_.push_back (*it);
	    }
	    joints_.push_back (commonAncestor);
	    indexCommonAncestor_ = joints_.size () - 1;
	    for (std::deque <JointPtr_t>::const_iterator it =
		   bAncestors.begin (); it != bAncestors.end (); ++it) {
		      joints_.push_back (*it);
	    }
      joints_.push_back (joint_b_);
	  }

	  void computeCoefficients ()
	  {
	    JointPtr_t child;
	    assert (joints_.size () > 1);
	    coefficients_.resize (joints_.size () - 1);
      pinocchio::DeviceConstPtr_t robot =joint_a_->robot ();
	    // Store r0 + sum of T_{i/i+1} in a variable
	    value_type cumulativeLength = joint_a_->linkedBody ()->radius ();
	    value_type distance;
	    size_type i = 0;
	    std::vector <JointPtr_t>::const_iterator it = joints_.begin ();
	    std::vector <JointPtr_t>::const_iterator itNext = it + 1;
	    while (itNext != joints_.end ()) {
	      if (robot->model ().parents[(*it)->index ()] == (*itNext)->index ()) {
		child = *it;
	      } else if (robot->model ().parents[(*itNext)->index ()] == (*it)->index ()) {
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
	      const JointPtr_t& joint = itCoef->joint_;
	      const value_type& value = itCoef->value_;
	      //maximalVelocity_ += value * joint->configuration ()->distance
		    //(q1, q2, joint->rankInConfiguration ()) / T;
        maximalVelocity_ += value * joint->robot ()->model ().joints[
            joint->index ()].distance_impl (q1, q2) / T;
	    }
	  }

	  JointPtr_t joint_a_;
	  JointPtr_t joint_b_;
	  std::vector<CollisionObjectConstPtr_t> objects_a_;
    std::vector<CollisionObjectConstPtr_t> objects_b_;
	  std::vector <JointPtr_t> joints_;
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
