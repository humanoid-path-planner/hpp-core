//
// Copyright (c) 2014,2015,2016 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/collision-object.hh>
# include <hpp/pinocchio/joint.hh>
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
              const value_type& t1, ConfigurationIn_t q1) const
	  {
	    const value_type T = t1 - t0;
            if (T == 0) return std::numeric_limits<value_type>::infinity();

	    value_type maximalVelocity = 0;
            for (CoefficientVelocities_t::const_iterator itCoef =
                coefs_->begin (); itCoef != coefs_->end (); ++itCoef) {
              const JointPtr_t& joint = itCoef->joint_;
              const value_type& value = itCoef->value_;
              maximalVelocity += value * joint->robot ()->model ().joints[
                joint->index ()].distance (q0, q1) / T;
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
				      CollisionValidationReportPtr_t& report)
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
	    for (ConstObjectStdVector_t::const_iterator ita = objects_a_.begin ();
                ita != objects_a_.end (); ++ita) {
              // Compute position of object a
              pinocchio::FclConstCollisionObjectPtr_t object_a = (*ita)->fcl ();
              for (ConstObjectStdVector_t::const_iterator itb = objects_b_.begin ();
                  itb != objects_b_.end (); ++itb) {
                // Compute position of object b
                pinocchio::FclConstCollisionObjectPtr_t object_b = (*itb)->fcl ();
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

          std::ostream& print (std::ostream& os) const
          {
            os << "Progressive BodyPairCollision: " << joint_a_->name()
              << " - " << (joint_b_ ? joint_b_->name() : "World") << '\n';
            const se3::Model& model = joint_a_->robot ()->model();
            for (std::size_t i = 0; i < joints_.size (); ++i) {
              if (i > 0) std::cout << model.names[i] << ',';
              else       std::cout << "World"        << ',';
            }
            std::cout << '\n';
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
	    joint_a_ (joint_a), joint_b_ (), objects_a_ (),
	    objects_b_ (objects_b), joints_ (),
	    indexCommonAncestor_ (0), coefficients_ (),
            pathVelocity_ (&coefficients_),
	    tolerance_ (tolerance), reverse_ (false)
	  {
	    assert (joint_a);
	    BodyPtr_t body_a = joint_a_->linkedBody ();
	    assert (body_a);
      for (size_type i = 0; i < body_a->innerObjects ().size (); ++i) {
	        objects_a_.push_back (body_a->innerObjects ().at(i));
      }
	    for (std::vector<CollisionObjectConstPtr_t>::const_iterator it = objects_b.begin ();
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
            pinocchio::DevicePtr_t robot =joint_a_->robot ();
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

	  JointPtr_t joint_a_;
	  JointPtr_t joint_b_;
	  ConstObjectStdVector_t objects_a_;
	  ConstObjectStdVector_t objects_b_;
	  std::vector <JointIndex> joints_;
	  std::size_t indexCommonAncestor_;
	  CoefficientVelocities_t coefficients_;
          PathPtr_t path_;
	  PathVelocity pathVelocity_;
	  value_type tolerance_;
	  bool valid_;
	  bool reverse_;
	}; // class BodyPairCollision

        inline std::ostream& operator<< (std::ostream& os, const BodyPairCollision& b)
        {
          return b.print (os);
        }
      } // namespace progressive
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONT_COLLISION_CHECKING_PROGRESSIVE_BODY_PAIR_COLLISION_HH
