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

#ifndef HPP_CORE_CONT_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH
# define HPP_CORE_CONT_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH

# include <limits>
# include <iterator>

# include <boost/icl/continuous_interval.hpp>
# include <boost/icl/interval_set.hpp>

# include <hpp/fcl/collision_data.h>
# include <hpp/fcl/collision.h>
# include <pinocchio/multibody/model.hpp>
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/collision-object.hh>
# include <hpp/pinocchio/joint.hh>
# include <hpp/pinocchio/joint-collection.hh>
# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/interpolated-path.hh>
# include <hpp/core/deprecated.hh>
# include "continuous-collision-checking/intervals.hh"
# include "continuous-collision-checking/path-velocity.hh"

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      HPP_PREDEF_CLASS (BodyPairCollision);
      typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;

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
        typedef std::pair<CollisionObjectConstPtr_t, CollisionObjectConstPtr_t> CollisionPair_t;
        typedef std::vector<CollisionPair_t> CollisionPairs_t;

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


	const std::vector <pinocchio::JointIndex>& joints () const
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

	const CollisionPairs_t& pairs () const
	{
	  return pairs_;
	}

	bool removeObjectTo_b (const CollisionObjectConstPtr_t& object)
	{
          const std::size_t s = pairs_.size();
	  for (CollisionPairs_t::iterator _pair = pairs_.begin ();
	       _pair != pairs_.end (); ) {
	    if (object == _pair->second) _pair = pairs_.erase (_pair);
	    else                         ++_pair;
	  }
	  return pairs_.size() < s;
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
          validInterval_ = interval_set();
	}

	/// Get path
	PathConstPtr_t path () const
	{
	  return path_;
	}

	/// Validate interval centered on a path parameter
	/// \param t parameter value in the path interval of definition
	/// \param[in,out] interval as input, interval over which
        ///                collision checking must be performed.
        ///                As output, interval over which pair is collision-free,
        ///                not necessarily included in definition interval.
	/// \return true if the body pair is collision free for this parameter
	///         value, false if the body pair is in collision.
	/// \note object should be in the positions defined by the configuration
	///       of parameter t on the path.
	bool validateConfiguration (const value_type& t, interval_t& interval,
				    CollisionValidationReportPtr_t& report)
	{
          namespace icl = boost::icl;
	  if (valid_) {
	    interval = path_->timeRange ();
	    return true;
	  }
          continuous_interval iclInterval
            (interval.first, interval.second, icl::interval_bounds::closed());
          if (icl::contains (validInterval_, iclInterval))
          {
            // TODO interval could probably be enlarge using validInterval_
            // interval = validInterval_;
            return true;
          }
	  using std::numeric_limits;
	  value_type distanceLowerBound =
	    numeric_limits <value_type>::infinity ();
          static const fcl::CollisionRequest request
            (1, false, true, 1, false, true, fcl::GST_INDEP);
          for (CollisionPairs_t::const_iterator _pair = pairs_.begin();
              _pair != pairs_.end(); ++_pair) {
	    pinocchio::FclConstCollisionObjectPtr_t object_a = _pair->first ->fcl ();
            pinocchio::FclConstCollisionObjectPtr_t object_b = _pair->second->fcl ();
            fcl::CollisionResult result;
            fcl::collide (object_a, object_b, request, result);
            // Get result
            if (result.isCollision ()) {
              report = CollisionValidationReportPtr_t
                (new CollisionValidationReport);
              report->object1 = _pair->first ;
              report->object2 = _pair->second;
              report->result = result;
              return false;
            }
            if (result.distance_lower_bound < distanceLowerBound) {
              distanceLowerBound = result.distance_lower_bound;
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
            value_type Vm;
            halfLengthDist = pathVelocity_.collisionFreeInterval(t, distanceLowerBound, Vm);
	    halfLengthTol = 2*tolerance_/Vm;
	  }
	  assert (!isnan (halfLengthDist));
	  assert (!isnan (halfLengthTol));
	  interval.first = t - (halfLengthDist + halfLengthTol);
	  interval.second = t + (halfLengthDist + halfLengthTol);

          validInterval_.insert (continuous_interval(interval.first, interval.second, icl::interval_bounds::closed()));

          // Check if the whole path is valid.
          iclInterval = continuous_interval (path_->timeRange ().first, path_->timeRange ().second, icl::interval_bounds::closed());
          if (icl::contains (validInterval_, iclInterval))
	    valid_ = true;
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
	  else oss << "obstacles";
	  oss << ")";
	  return oss.str ();
	}

	std::ostream& print (std::ostream& os) const
	{
	  os << "Progressive BodyPairCollision: " << joint_a_->name()
	     << " - " << (joint_b_ ? joint_b_->name() : "World") << '\n';
	  const pinocchio::Model& model = joint_a_->robot ()->model();
	  for (std::size_t i = 0; i < joints_.size (); ++i) {
	    if (i > 0) os << model.names[i] << ',';
	    else       os << "World"        << ',';
	  }
	  os << '\n';
	  for (std::size_t i = 0; i < coefficients_.size(); ++i)
	    os << coefficients_[i].value_ << ", ";
	  return os;
	}

        /// \note The left object should belong to joint_a and
        /// the right one should belong to joint_b, or vice-versa.
        /// This is not checked.
        void addCollisionPair (const CollisionObjectConstPtr_t& left,
            const CollisionObjectConstPtr_t right)
        {
          pairs_.push_back (CollisionPair_t(left, right));
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
	  joint_a_ (joint_a), joint_b_ (joint_b), pairs_ (), joints_ (),
	  indexCommonAncestor_ (0), coefficients_ (),
	  pathVelocity_ (&coefficients_),
	  tolerance_ (tolerance), reverse_ (false)
	{
	  assert (joint_a);
	  assert (joint_b);
	  if (joint_b_->robot () != joint_a_->robot ()) {
	    throw std::runtime_error
	      ("Joints do not belong to the same device.");
	  }
	  if (joint_a_->index() == joint_b_->index()) {
	    throw std::runtime_error ("Bodies should be different");
	  }
	  if (tolerance < 0) {
	    throw std::runtime_error ("tolerance should be non-negative.");
	  }

	  BodyPtr_t body_a = joint_a_->linkedBody ();
	  BodyPtr_t body_b = joint_b_->linkedBody ();
	  assert (body_a);
	  assert (body_b);

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
	  joint_a_ (joint_a), joint_b_ (), pairs_ (), joints_ (),
	  indexCommonAncestor_ (0), coefficients_ (),
	  pathVelocity_ (&coefficients_),
	  tolerance_ (tolerance), reverse_ (false)
	{
	  if (tolerance < 0) {
	    throw std::runtime_error ("tolerance should be non-negative.");
	  }
	  assert (joint_a);
	  BodyPtr_t body_a = joint_a_->linkedBody ();
	  assert (body_a);
          for (size_type i = 0; i < body_a->nbInnerObjects(); ++i) {
	    CollisionObjectConstPtr_t obj = body_a->innerObjectAt(i);

            for (ConstObjectStdVector_t::const_iterator it = objects_b.begin ();
                it != objects_b.end (); ++it) {
              assert (!(*it)->joint () ||
                  (*it)->joint ()->robot () != joint_a_->robot ());
              pairs_.push_back (CollisionPair_t(obj, *it));
            }
	  }

	  // Find sequence of joints
	  computeSequenceOfJoints ();
	  computeCoefficients ();
	}

      private:
	typedef pinocchio::JointIndex JointIndex;

	void computeSequenceOfJoints ()
	{
	  joints_.clear ();

	  const pinocchio::Model& model = joint_a_->robot ()->model();
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
	  const pinocchio::Model& model = joint_a_->robot ()->model();

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

        typedef boost::icl::continuous_interval<value_type> continuous_interval;
        typedef boost::icl::interval_set<value_type> interval_set;

	JointPtr_t joint_a_;
	JointPtr_t joint_b_;
        CollisionPairs_t pairs_;
	std::vector <JointIndex> joints_;
	std::size_t indexCommonAncestor_;
	CoefficientVelocities_t coefficients_;
	PathPtr_t path_;
	PathVelocity pathVelocity_;
	value_type tolerance_;
	bool valid_;
        interval_set validInterval_;
	bool reverse_;
      }; // class BodyPairCollision

      inline std::ostream& operator<< (std::ostream& os, const BodyPairCollision& b)
      {
	return b.print (os);
      }
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONT_COLLISION_CHECKING_BODY_PAIR_COLLISION_HH
