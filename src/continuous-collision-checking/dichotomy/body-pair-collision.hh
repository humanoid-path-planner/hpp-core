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
	    BodyPairCollisionPtr_t shPtr (new BodyPairCollision
					  (joint_a, objects_b, tolerance));
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


	  const std::vector <se3::JointIndex>& joints () const
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

	  const ConstObjectStdVector_t& objects_b  () const
	  {
	    return objects_b_;
	  }

	  bool removeObjectTo_b (const CollisionObjectConstPtr_t& object)
	  {
	    for (ConstObjectStdVector_t::iterator itObj = objects_b_.begin ();
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
            static const fcl::CollisionRequest request (1, false, true, 1,
                false, true, fcl::GST_INDEP);
            fcl::CollisionResult result;

            // const se3::Model& model = joint_a_->robot ()->model();
            // FIXME Here is a good place to have a local Data.
            const se3::Data&  data  = joint_a_->robot ()->data();

            DevicePtr_t robot = joint_a_->robot ();
            using std::numeric_limits;
            // Get configuration of robot corresponding to parameter
            bool success;
            const Configuration_t q = (*path_) (t, success);
            if (!success) throw
              projection_error(std::string ("Unable to apply constraints in ") + __PRETTY_FUNCTION__);
	    // Compute position of joint a in frame of common ancestor
	    Transform3f Ma;
            Ma.setIdentity ();
            const Configuration_t& qSave = robot->currentConfiguration ();
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
              Ma = data.oMi[joints_ [(std::size_t)i]];
            }
            // Compute position of joint b in frame of common ancestor
            Transform3f Mb;
            Mb.setIdentity ();
            for (std::size_t i = indexCommonAncestor_ + 1; i < joints_.size (); ++i) {
              // joints_ [i]->computePosition (q, Mb, tmp);
              Ma = data.oMi[joints_ [(std::size_t)i]];
            }
            value_type distanceLowerBound =
              numeric_limits <value_type>::infinity ();
            for (ConstObjectStdVector_t::const_iterator ita = objects_a_.begin ();
                ita != objects_a_.end (); ++ita) {
              // Compute position of object a
              pinocchio::FclCollisionObjectPtr_t object_a =
                const_cast<fcl::CollisionObject*> ((*ita)->fcl ());
              object_a->setTransform (se3::toFclTransform3f(Ma * (*ita)->positionInJointFrame ()));
              for (ConstObjectStdVector_t::const_iterator itb = objects_b_.begin ();
                  itb != objects_b_.end (); ++itb) {
                // Compute position of object b
                pinocchio::FclCollisionObjectPtr_t object_b =
                  const_cast<fcl::CollisionObject*> ((*itb)->fcl ());
                object_b->setTransform (se3::toFclTransform3f(Mb * (*itb)->positionInJointFrame ()));
                // Perform collision test
                fcl::collide (object_a, object_b, request, result);
                // TODO: where should the configuration be set back to original?
                robot->currentConfiguration (qSave);
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

          std::ostream& print (std::ostream& os) const
          {
            os << "Dichotomy BodyPairCollision: " << joint_a_->name()
              << " - " << (joint_b_ ? joint_b_->name() : "World") << '\n';
            const se3::Model& model = joint_a_->robot ()->model();
            for (std::size_t i = 0; i < joints_.size (); ++i) {
              if (i > 0) os << model.names[i] << ',';
              else       os << "World"        << ',';
            }
            os << '\n';
            for (std::size_t i = 0; i < coefficients_.size(); ++i)
              os << coefficients_[i].value_ << ", ";
            return os;
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
      for (size_type i = 0; i < body_a->innerObjects ().size (); ++i) {
	    objects_a_.push_back (body_a->innerObjects ().at(i));
      }
      for (size_type i = 0; i < body_b->innerObjects ().size (); ++i) {
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
			     const ConstObjectStdVector_t& objects_b,
			     value_type tolerance) :
	    joint_a_ (joint_a), joint_b_ (), objects_a_ (), objects_b_ (),
	    joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (), maximalVelocity_ (0),
	    tolerance_ (tolerance)
	  {
	    assert (joint_a);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    assert (body_a);
      for (size_type i = 0; i < body_a->innerObjects ().size (); ++i) {
	        objects_a_.push_back (body_a->innerObjects ().at(i));
      }
	    for (ConstObjectStdVector_t::const_iterator it = objects_b.begin ();
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
          typedef se3::JointIndex JointIndex;

	  void computeSequenceOfJoints ()
          {
            joints_.clear ();

            const se3::Model& model = joint_a_->robot ()->model();
            const JointIndex id_a = joint_a_->index(),
                             id_b = (joint_b_ ? joint_b_->index() : 0);
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
            joints_ = fromA;
            joints_.insert(joints_.end(), fromB.rbegin(), fromB.rend());
            indexCommonAncestor_ = fromA.size() - 1;
            assert(joints_.front() == id_a);
            assert(joints_.back() == id_b);
            assert(joints_.size() > 1);
          }

	  void computeCoefficients ()
	  {
            const se3::Model& model = joint_a_->robot ()->model();

	    JointPtr_t child;
	    assert (joints_.size () > 1);
            coefficients_.resize (joints_.size () - 1);
            DevicePtr_t robot = joint_a_->robot ();
	    // Store r0 + sum of T_{i/i+1} in a variable
	    value_type cumulativeLength = joint_a_->linkedBody ()->radius ();
	    value_type distance;
            std::size_t i = 0;
	    while (i + 1 < joints_.size()) {
	      if (model.parents[joints_[i]] == joints_[i+1])
                child = JointPtr_t (new Joint(robot, joints_[i]));
	      else if (model.parents[joints_[i+1]] == joints_[i])
                child = JointPtr_t (new Joint(robot, joints_[i+1]));
	      else
                abort ();
	      coefficients_ [i].joint_ = child;
	      // Go through all known types of joints
	    //  TODO: REPLACE THESE FUNCTIONS WITH NEW API
              distance = child->maximalDistanceToParent ();
              coefficients_ [i].value_ =
                child->upperBoundLinearVelocity () +
                cumulativeLength * child->upperBoundAngularVelocity ();
              cumulativeLength += distance;

	      ++i;
	    }
	  }

	  /// Compute maximal velocity of points of body1 in the frame of body 2
	  /// \param path input path
	  void computeMaximalVelocity ()
	  {
            const se3::Model& model = joint_a_->robot ()->model();

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
              JointIndex jid = itCoef->joint_->index();
	      const value_type& value = itCoef->value_;
	      //maximalVelocity_ += value * joint->configuration ()->distance
		    //(q1, q2, joint->rankInConfiguration ()) / T;
              maximalVelocity_ += value * model.joints[jid].distance (q1, q2) / T;
            }
	  }

	  JointPtr_t joint_a_;
	  JointPtr_t joint_b_;
          ConstObjectStdVector_t objects_a_;
          ConstObjectStdVector_t objects_b_;
	  std::vector <JointIndex> joints_;
	  std::size_t indexCommonAncestor_;
	  std::vector <CoefficientVelocity> coefficients_;
	  StraightPathPtr_t path_;
	  value_type maximalVelocity_;
	  Intervals intervals_;
	  value_type tolerance_;
	}; // class BodyPairCollision

        inline std::ostream& operator<< (std::ostream& os, const BodyPairCollision& b)
        {
          return b.print (os);
        }
      } // namespace dichotomy
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONT_COLLISION_CHECKING_DICHOTOMY_BODY_PAIR_COLLISION_HH
