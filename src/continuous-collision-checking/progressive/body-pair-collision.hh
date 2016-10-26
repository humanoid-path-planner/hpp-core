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

#ifndef HPP_CORE_CONT_COLLISION_CHECKING_PROGRESSIVE_BODY_PAIR_COLLISION_HH
# define HPP_CORE_CONT_COLLISION_CHECKING_PROGRESSIVE_BODY_PAIR_COLLISION_HH

# include <limits>
# include <iterator>

# include <hpp/fcl/collision_data.h>
# include <hpp/fcl/collision.h>
# include <hpp/model/body.hh>
# include <hpp/model/collision-object.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/interpolated-path.hh>
# include <hpp/core/deprecated.hh>
# include "continuous-collision-checking/intervals.hh"


namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      namespace progressive {
	struct PathVelocity;
	HPP_PREDEF_CLASS (BodyPairCollision);
	typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;
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
        typedef std::vector <CoefficientVelocity> CoefficientVelocities_t;

	struct PathVelocity
	{
          typedef std::map <value_type, value_type> Velocities_t;

          PathVelocity (CoefficientVelocities_t const* coefs, PathPtr_t path) :
            coefs_ (coefs)
          {
            StraightPathPtr_t sp = HPP_DYNAMIC_PTR_CAST (StraightPath, path);
            if (sp) { init (sp); return; }
            InterpolatedPathPtr_t ip = HPP_DYNAMIC_PTR_CAST (InterpolatedPath, path);
            if (ip) { init (ip); return; }
            throw std::logic_error ("Unknown type of paths");
          }

          PathVelocity (CoefficientVelocities_t const* coefs) :
            maximalVelocity_ (0), coefs_ (coefs) {}

          void init (StraightPathPtr_t path)
          {
	    value_type t0 = path->timeRange ().first;
	    value_type t1 = path->timeRange ().second;
            Configuration_t q0 = path->initial();
            Configuration_t q1 = path->end();
            maximalVelocity_ = computeMaximalVelocity (t0, q0, t1, q1);
            maximalVelocities_.insert(std::make_pair (t1, maximalVelocity_));
          }

          void init (InterpolatedPathPtr_t path)
          {
            typedef InterpolatedPath::InterpolationPoints_t IPs_t;
            const IPs_t& ips = path->interpolationPoints();
            value_type tprev = path->timeRange ().first;
            Configuration_t qprev = path->initial();
            maximalVelocity_ = 0;
            for (IPs_t::const_iterator it = (++ips.begin ());
                it != ips.end(); ++it) {
              const value_type& t = it->first;
              const Configuration_t& q = it->second;
              value_type mv = computeMaximalVelocity (tprev, qprev, t, q);
              maximalVelocities_.insert(std::make_pair (t, mv));
              if (mv > maximalVelocity_) maximalVelocity_ = mv;
              tprev = t;
              qprev = q;
            }
          }

	  value_type maximalVelocity (const value_type& t) const
	  {
            Velocities_t::const_iterator itAfter =
              maximalVelocities_.lower_bound(t);
            if (itAfter != maximalVelocities_.begin ()) itAfter--; 
            return itAfter->second;
	  }

	  /// Compute maximal velocity of points of body1 in the frame of body 2
	  /// \param path input path
	  value_type computeMaximalVelocity (
              const value_type& t0, ConfigurationIn_t q0,
              const value_type& t1, ConfigurationIn_t q1)
	  {
	    const value_type T = t1 - t0;
            if (T == 0) return std::numeric_limits<value_type>::infinity();

	    value_type maximalVelocity = 0;
	    for (CoefficientVelocities_t::const_iterator itCoef =
		   coefs_->begin (); itCoef != coefs_->end (); ++itCoef) {
	      const JointConstPtr_t& joint = itCoef->joint_;
	      const value_type& value = itCoef->value_;
	      maximalVelocity += value * joint->configuration ()->distance
		(q0, q1, joint->rankInConfiguration ()) / T;
	    }
            return maximalVelocity;
	  }

	  Velocities_t maximalVelocities_;
          value_type maximalVelocity_;
          CoefficientVelocities_t const* coefs_;
	}; // struct PathVelocity

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
	      throw std::runtime_error
		("Object should not be attached to a joint"
		 " to add it to a collision pair.");
	    }
	    objects_b_.push_back (object);
	  }

	  const ObjectVector_t& objects_b  () const
	  {
	    return objects_b_;
	  }

	  bool removeObjectTo_b (const CollisionObjectPtr_t& object)
	  {
	    for (ObjectVector_t::iterator itObj = objects_b_.begin ();
		 itObj != objects_b_.end (); ++itObj) {
	      if (object == *itObj) {
		objects_b_.erase (itObj);
		return true;
	      }
	    }
	    return false;
	  }

	  /// Set path to validate
	  /// \param path path to validate,
	  /// \param reverse whether path is validated from end to beginning.
	  /// Compute maximal velocity of point of body a in frame of body b
	  /// along the path.
	  void path (const PathPtr_t& path, bool reverse)
          {
            path_ = path;
            pathVelocity_ = PathVelocity (&coefficients_, path);
            reverse_ = reverse;
            valid_ = false;
          }

	  /// Get path
	  PathConstPtr_t path () const
	  {
	    return path_;
	  }

	  /// Validate interval centered on a path parameter
	  /// \param t parameter value in the path interval of definition
	  /// \return true if the body pair is collision free for this parameter
	  ///         value, false if the body pair is in collision.
	  bool validateConfiguration (const value_type& t, value_type& tmin,
				      CollisionValidationReport& report)
	    HPP_CORE_DEPRECATED
	  {
	    if (valid_) {
	      if (reverse_) {
		tmin = path_->timeRange ().first;
	      } else {
		tmin = path_->timeRange ().second;
	      }
	      return true;
	    }
	    using std::numeric_limits;
	    value_type distanceLowerBound =
	      numeric_limits <value_type>::infinity ();
	    for (ObjectVector_t::const_iterator ita = objects_a_.begin ();
		 ita != objects_a_.end (); ++ita) {
	      // Compute position of object a
	      fcl::CollisionObject* object_a = (*ita)->fcl ().get ();
	      for (ObjectVector_t::const_iterator itb = objects_b_.begin ();
		   itb != objects_b_.end (); ++itb) {
		// Compute position of object b
		fcl::CollisionObject* object_b = (*itb)->fcl ().get ();
		// Perform collision test
		fcl::CollisionRequest request (1, false, true, 1, false, true,
					       fcl::GST_INDEP);
		fcl::CollisionResult result;
		fcl::collide (object_a, object_b, request, result);
		// Get result
		if (result.isCollision ()) {
		  hppDout (info, "collision at " << t << " for pair ("
			   << joint_a_->name () << "," << (*itb)->name ()
			   << ")");
		  report.object1 = *ita;
		  report.object2 = *itb;
		  return false;
		}
		if (result.distance_lower_bound < distanceLowerBound) {
		  distanceLowerBound = result.distance_lower_bound;
		}
	      }
	    }
	    value_type halfLengthDist, halfLengthTol;
	    if (distanceLowerBound ==
		numeric_limits <value_type>::infinity ()) {
	      halfLengthDist = numeric_limits <value_type>::infinity ();
	      halfLengthTol = 0;
	    } else {
	      halfLengthDist = distanceLowerBound/pathVelocity_.maximalVelocity_;
	      halfLengthTol = 2*tolerance_/pathVelocity_.maximalVelocity_;
	    }
	    assert (!isnan (halfLengthDist));
	    assert (!isnan (halfLengthTol));
	    if (reverse_) {
	      tmin = t - (halfLengthDist + halfLengthTol);
	      if (t - halfLengthDist <= path_->timeRange ().first) {
		valid_ = true;
	      }
	    } else {
	      tmin = t + halfLengthDist + halfLengthTol;
	      if (t + halfLengthDist >= path_->timeRange ().second) {
		valid_ = true;
	      }
	    }
	    std::string joint2;
	    if (joint_b_) joint2 = joint_b_->name ();
	    else joint2 = (*objects_b_.begin ())->name ();
	    return true;
	  }

	  /// Validate interval centered on a path parameter
	  /// \param t parameter value in the path interval of definition
	  /// \return true if the body pair is collision free for this parameter
	  ///         value, false if the body pair is in collision.
	  bool validateConfiguration (const value_type& t, value_type& tmin,
				      CollisionValidationReportPtr_t& report)
	  {
	    if (valid_) {
	      if (reverse_) {
		tmin = path_->timeRange ().first;
	      } else {
		tmin = path_->timeRange ().second;
	      }
        hppDout(info,"tmin = "<<tmin);
	      return true;
	    }
	    using std::numeric_limits;
	    value_type distanceLowerBound =
	      numeric_limits <value_type>::infinity ();
	    for (ObjectVector_t::const_iterator ita = objects_a_.begin ();
		 ita != objects_a_.end (); ++ita) {
	      // Compute position of object a
	      fcl::CollisionObject* object_a = (*ita)->fcl ().get ();
	      for (ObjectVector_t::const_iterator itb = objects_b_.begin ();
		   itb != objects_b_.end (); ++itb) {
		// Compute position of object b
		fcl::CollisionObject* object_b = (*itb)->fcl ().get ();
		// Perform collision test
		fcl::CollisionRequest request (1, false, true, 1, false, true,
					       fcl::GST_INDEP);
		fcl::CollisionResult result;
		fcl::collide (object_a, object_b, request, result);
		// Get result
		if (result.isCollision ()) {
		  hppDout (info, "collision at " << t << " for pair ("
			   << joint_a_->name () << "," << (*itb)->name ()
			   << ")");
		  report = CollisionValidationReportPtr_t
		    (new CollisionValidationReport);
		  report->object1 = *ita;
		  report->object2 = *itb;
		  report->result = result;
		  return false;
		}
		if (result.distance_lower_bound < distanceLowerBound) {
		  distanceLowerBound = result.distance_lower_bound;
		}
	      }
	    }
	    value_type halfLengthDist, halfLengthTol;
            /// \todo A finer bound could be computed when path is an
            ///       InterpolatedPath using the maximal velocity on each
            ///       subinterval
	    if (distanceLowerBound ==
		numeric_limits <value_type>::infinity ()) {
	      halfLengthDist = numeric_limits <value_type>::infinity ();
	      halfLengthTol = 0;
	    } else {
	      halfLengthDist = distanceLowerBound/pathVelocity_.maximalVelocity_;
	      halfLengthTol = 2*tolerance_/pathVelocity_.maximalVelocity_;
	    }
	    assert (!isnan (halfLengthDist));
	    assert (!isnan (halfLengthTol));
	    if (reverse_) {
	      tmin = t - (halfLengthDist + halfLengthTol);
	      if (t - halfLengthDist <= path_->timeRange ().first) {
		valid_ = true;
	      }
	    } else {
	      tmin = t + halfLengthDist + halfLengthTol;
	      if (t + halfLengthDist >= path_->timeRange ().second) {
		valid_ = true;
	      }
	    }
	    std::string joint2;
	    if (joint_b_) joint2 = joint_b_->name ();
	    else joint2 = (*objects_b_.begin ())->name ();
      hppDout(info,"tmin = "<<tmin);
	    return true;
	  }

	  value_type tolerance () const
	  {
	    return tolerance_;
	  }

	  value_type maximalVelocity () const
	  {
	    return pathVelocity_.maximalVelocity_;
	  }

	  std::string name () const
	  {
	    std::ostringstream oss;
	    oss << "(" << joint_a_->name () << ",";
	    if (joint_b_) oss << joint_b_->name ();
	    else oss << (*objects_b_.begin ())->name ();
	    oss << ")";
	    return oss.str ();
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
	    indexCommonAncestor_ (0), coefficients_ (),
            pathVelocity_ (&coefficients_),
	    tolerance_ (tolerance), reverse_ (false)
	  {
	    assert (joint_a);
	    assert (joint_b);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    BodyPtr_t body_b = joint_b_->linkedBody ();
	    assert (body_a);
	    assert (body_b);
	    objects_a_ = body_a->innerObjects (model::COLLISION);
	    objects_b_ = body_b->innerObjects (model::COLLISION);

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
	    joint_a_ (joint_a), joint_b_ (), objects_a_ (),
	    objects_b_ (objects_b), joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (),
            pathVelocity_ (&coefficients_),
	    tolerance_ (tolerance), reverse_ (false)
	  {
	    assert (joint_a);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    assert (body_a);
	    objects_a_ = body_a->innerObjects (model::COLLISION);
	    for (ObjectVector_t::const_iterator it = objects_b.begin ();
		 it != objects_b.end (); ++it) {
	      assert (!(*it)->joint () ||
		      (*it)->joint ()->robot () != joint_a_->robot ());
	    }
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
	      distance = child->maximalDistanceToParent ();
	      coefficients_ [i].value_ = child->upperBoundLinearVelocity () +
		cumulativeLength * child->upperBoundAngularVelocity ();
	      cumulativeLength += distance;
	      it = itNext; ++itNext; ++i;
	    }
	  }

	  JointConstPtr_t joint_a_;
	  JointConstPtr_t joint_b_;
	  ObjectVector_t objects_a_;
	  ObjectVector_t objects_b_;
	  std::vector <JointConstPtr_t> joints_;
	  std::size_t indexCommonAncestor_;
	  CoefficientVelocities_t coefficients_;
          PathPtr_t path_;
	  PathVelocity pathVelocity_;
	  value_type tolerance_;
	  bool valid_;
	  bool reverse_;
	}; // class BodyPairCollision
      } // namespace progressive
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONT_COLLISION_CHECKING_PROGRESSIVE_BODY_PAIR_COLLISION_HH
