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

#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/straight-path.hh>

#include "continuous-collision-checking/progressive/body-pair-collision.hh"

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {

      using progressive::BodyPairCollision;
      using progressive::BodyPairCollisionPtr_t;
      using progressive::BodyPairCollisions_t;

      bool compareBodyPairCol (const BodyPairCollisionPtr_t& bodyPair1,
			       const BodyPairCollisionPtr_t& bodyPair2)
      {
	assert (!bodyPair1->validSubset ().list ().empty ());
	assert (!bodyPair2->validSubset ().list ().empty ());
	return bodyPair1->validSubset ().list ().begin ()->second <
	  bodyPair2->validSubset ().list ().begin ()->second;
      }

      bool compareReverseBodyPairCol (const BodyPairCollisionPtr_t& bodyPair1,
				      const BodyPairCollisionPtr_t& bodyPair2)
      {
	assert (!bodyPair1->validSubset ().list ().empty ());
	assert (!bodyPair2->validSubset ().list ().empty ());
	return bodyPair1->validSubset ().list ().rbegin ()->first >
	  bodyPair2->validSubset ().list ().rbegin ()->first;
      }

      // Compute last parameter that is collision free without tolerance
      value_type computeLastValidParameter
      (const BodyPairCollisions_t& bodyPairCollisions, bool reverse)
      {
	// For each pair, remove the interval range corresponding to penetration
	// to the upper bound of the first interval and find the minimum of
	// the resulting upper bounds.
	if (reverse) {
	  value_type tmax = - std::numeric_limits <value_type>::infinity ();
	  for (BodyPairCollisions_t::const_iterator itPair =
		 bodyPairCollisions.begin ();
	       itPair != bodyPairCollisions.end (); ++itPair) {
	    value_type t = (*itPair)->lastValidParameter ();
	    if (t > tmax) tmax = t;
	  }
	  return tmax;
	} else {
	  std::string pairName;
	  value_type maximalVelocity;
	  value_type tmin = std::numeric_limits <value_type>::infinity ();
	  for (BodyPairCollisions_t::const_iterator itPair =
		 bodyPairCollisions.begin ();
	       itPair != bodyPairCollisions.end (); ++itPair) {
	    value_type t = (*itPair)->lastValidParameter ();
	    if (t < tmin) {
	      tmin = t;
	      pairName = (*itPair)->name ();
	      maximalVelocity = (*itPair)->maximalVelocity ();
	    }
	  }
	  hppDout (info, "Last valid parameter for pair " << pairName
		   << ", maximal velocity=" << maximalVelocity);
	  return tmin;
	}
      }

      ProgressivePtr_t Progressive::create (const DevicePtr_t& robot,
					    const value_type& tolerance)
      {
	Progressive* ptr =
	  new Progressive (robot, tolerance);
	ProgressivePtr_t shPtr (ptr);
	return shPtr;
      }

      bool Progressive::validate
      (const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
      {
	StraightPathPtr_t straightPath = HPP_DYNAMIC_PTR_CAST
	  (StraightPath, path);
	// for each BodyPairCollision
	//   - set path,
	//   - compute valid interval at start (end if reverse)
	value_type t0 = path->timeRange ().first;
	value_type t1 = path->timeRange ().second;
	value_type lower, upper;
	Configuration_t q;
	value_type toleranceRange = 0;
	bool collision = false;
	if (reverse) {
	  q = (*path) (t1);
	  robot_->currentConfiguration (q);
	  robot_->computeForwardKinematics ();
	  BodyPairCollisions_t::iterator itPair;
	  for (itPair = bodyPairCollisions_.begin ();
	       itPair != bodyPairCollisions_.end (); ++itPair) {
	    (*itPair)->path (straightPath, reverse);
	    // If collision at start point, return false
	    bool valid = (*itPair)->validateInterval (t1);
	    if (!valid) {
	      validPart = path->extract (interval_t (t1, t1));
	      hppDout (error, "invalid initial configuration");
	      return false;
	    }
	    assert ((*itPair)->validSubset ().contains (t1));
	  }
	  // Sort collision pairs
	  bodyPairCollisions_.sort (compareReverseBodyPairCol);
	  BodyPairCollisionPtr_t first = *(bodyPairCollisions_.begin ());
	  bool finished = first->validSubset ().contains (t0);
	  value_type firstCollision = t0;
	  while (!finished) {
	    // find middle of first non valid interval
	    const std::list <interval_t>& intervals =
	      first->validSubset ().list ();
	    std::list <interval_t>::const_reverse_iterator firstInterval =
	      intervals.rbegin ();
	    std::list <interval_t>::const_reverse_iterator secondInterval =
	      firstInterval;
	    ++secondInterval;
	    upper = firstInterval->first;
	    if (secondInterval != intervals.rend ()) {
	      lower = secondInterval->first;
	    } else {
	      lower = t0;
	    }
	    lower = std::max (lower, firstCollision);
	    assert (lower < upper);
	    value_type middle = .5 * (lower + upper);
	    assert (firstCollision < middle);
	    q = (*path) (middle);
	    robot_->currentConfiguration (q);
	    robot_->computeForwardKinematics ();
	    for (itPair = bodyPairCollisions_.begin ();
		 itPair != bodyPairCollisions_.end (); ++itPair) {
	      if (!(*itPair)->validSubset ().contains (middle) &&
		  !(*itPair)->validateInterval (middle)) {
		// collision
		collision = true;
		assert ((*itPair)->maximalVelocity ());
		toleranceRange = tolerance_ / (*itPair)->maximalVelocity ();
		firstCollision = middle + toleranceRange;
		hppDout (info, "middle=" << middle << ", firstCollision="
			 << firstCollision << ", toleranceRange="
			 << toleranceRange);
	      }
	    }
	    bodyPairCollisions_.sort (compareReverseBodyPairCol);
	    first = *(bodyPairCollisions_.begin ());
	    value_type t2 = first->validSubset ().list ().rbegin ()->first;
	    finished = (t2 <= t0 || t2 < firstCollision);
	  }
	  if (collision) {
	    upper = computeLastValidParameter (bodyPairCollisions_, reverse);
	    validPart = path->extract (interval_t (upper, t1));
	    return false;
	  }
	} else { // reverse
	  q = (*path) (t0);
	  robot_->currentConfiguration (q);
	  robot_->computeForwardKinematics ();
	  BodyPairCollisions_t::iterator itPair;
	  for (itPair = bodyPairCollisions_.begin ();
	       itPair != bodyPairCollisions_.end (); ++itPair) {
	    (*itPair)->path (straightPath, reverse);
	    // If collision at start point, return false
	    bool valid = (*itPair)->validateInterval (t0);
	    if (!valid) {
	      validPart = path->extract (interval_t (t0, t0));
	      hppDout (error, "invalid initial configuration");
	      return false;
	    }
	    assert ((*itPair)->validSubset ().contains (t0));
	  }
	  // Sort collision pairs
	  bodyPairCollisions_.sort (compareBodyPairCol);
	  BodyPairCollisionPtr_t first = *(bodyPairCollisions_.begin ());
	  bool finished = first->validSubset ().contains (t1);
	  value_type firstCollision = t1;
	  while (!finished) {
	    // find middle of first non valid interval
	    const std::list <interval_t>& intervals =
	      first->validSubset ().list ();
	    std::list <interval_t>::const_iterator firstInterval =
	      intervals.begin ();
	    std::list <interval_t>::const_iterator secondInterval =
	      firstInterval;
	    ++secondInterval;
	    lower = firstInterval->second;
	    if (secondInterval != intervals.end ()) {
	      upper = secondInterval->first;
	    } else {
	      upper = t1;
	    }
	    upper = std::min (upper, firstCollision);
	    assert (lower < upper);
	    value_type middle = .5 * (lower + upper);
	    assert (middle < firstCollision);
	    q = (*path) (middle);
	    robot_->currentConfiguration (q);
	    robot_->computeForwardKinematics ();
	    for (itPair = bodyPairCollisions_.begin ();
		 itPair != bodyPairCollisions_.end (); ++itPair) {
	      if (!(*itPair)->validSubset ().contains (middle) &&
		  !(*itPair)->validateInterval (middle)) {
		// collision
		collision = true;
		assert ((*itPair)->maximalVelocity ());
		toleranceRange = tolerance_ / (*itPair)->maximalVelocity ();
		firstCollision = middle - toleranceRange;
		hppDout (info, "middle=" << middle << ", firstCollision="
			 << firstCollision << ", toleranceRange="
			 << toleranceRange);
	      }
	    }
	    bodyPairCollisions_.sort (compareBodyPairCol);
	    first = *(bodyPairCollisions_.begin ());
	    value_type t2 = first->validSubset ().list ().begin ()->second;
	    finished = (t2 >= t1 || firstCollision < t2);
	  }
	  if (collision) {
	    lower = computeLastValidParameter (bodyPairCollisions_, reverse);
	    hppDout (info, "lower=" << lower << ", firstCollision=" <<
		     firstCollision);
	    validPart = path->extract (interval_t (t0, lower));
	    hppDout (info, "return valid path defined on [" << t0
		     << "," << lower << "] out of [" << t0 << ","
		     << t1 << "]");
	    return false;
	  }
	}
	validPart = path;
	hppDout (info, "return completely valid path defined on [" << t0
		 << "," << t1 << "]");
	return true;
      }

      void Progressive::addObstacle
      (const CollisionObjectPtr_t& object)
      {
	const JointVector_t& jv = robot_->getJointVector ();
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); ++itJoint) {
	  BodyPtr_t body = (*itJoint)->linkedBody ();
	  if (body) {
	    ObjectVector_t objects;
	    objects.push_back (object);
	    bodyPairCollisions_.push_back
	      (BodyPairCollision::create (*itJoint, objects, tolerance_));
	  }
	}
      }

      void Progressive::removeObstacleFromJoint
      (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
      {
	for (BodyPairCollisions_t::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  if (!(*itPair)->joint_b () && (*itPair)->joint_a () == joint) {
	    (*itPair)->removeObjectTo_b (obstacle);
	    if ((*itPair)->objects_b ().empty ()) {
	      bodyPairCollisions_.erase (itPair);
	    }
	    return;
	  }
	}
      }

      Progressive::~Progressive ()
      {
      }

      Progressive::Progressive
      (const DevicePtr_t& robot, const value_type& tolerance) :
	robot_ (robot), tolerance_ (tolerance),
	bodyPairCollisions_ ()
      {
	if (tolerance <= 0) {
	  throw std::runtime_error
	    ("tolerance should be positive for"
	     " progressive continuous collision checking.");
	}
	typedef model::Device::CollisionPairs_t CollisionPairs_t;
	// Build body pairs for collision checking
	// First auto-collision
	const CollisionPairs_t& colPairs = robot->collisionPairs
	  (model::COLLISION);
	for (CollisionPairs_t::const_iterator itPair = colPairs.begin ();
	     itPair != colPairs.end (); ++itPair) {
	  bodyPairCollisions_.push_back (BodyPairCollision::create
					 (itPair->first, itPair->second,
					  tolerance_));
	}
      }
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
