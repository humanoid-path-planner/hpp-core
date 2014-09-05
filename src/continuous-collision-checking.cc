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
#include <hpp/core/continuous-collision-checking.hh>
#include <hpp/core/straight-path.hh>

#include "continuous-collision-checking/body-pair-collision.hh"

namespace hpp {
  namespace core {

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
	       bodyPairCollisions.begin (); itPair != bodyPairCollisions.end ();
	     ++itPair) {
	  std::list <interval_t>::const_reverse_iterator itInterval =
	    (*itPair)->validSubset ().list ().rbegin ();
	  assert (itInterval != (*itPair)->validSubset ().list ().rend ());
	  value_type t = itInterval->first +
	    (*itPair)->tolerance () / (*itPair)->maximalVelocity ();
	  if (t > tmax) tmax = t;
	}
	return tmax;
      } else {
	value_type tmin = std::numeric_limits <value_type>::infinity ();
	for (BodyPairCollisions_t::const_iterator itPair =
	       bodyPairCollisions.begin (); itPair != bodyPairCollisions.end ();
	     ++itPair) {
	  std::list <interval_t>::const_iterator itInterval =
	    (*itPair)->validSubset ().list ().begin ();
	  assert (itInterval != (*itPair)->validSubset ().list ().end ());
	  value_type t = itInterval->second -
	    (*itPair)->tolerance () / (*itPair)->maximalVelocity ();
	  assert (t > 0);
	  if (t < tmin) tmin = t;
	}
	return tmin;
      }
    }

    // Partially sort a list where only the first element is not sorted
    template <typename Compare>
    void partialSort (std::list <BodyPairCollisionPtr_t>& list, Compare comp)
    {
      std::list <BodyPairCollisionPtr_t>::iterator it1 = list.begin ();
      if (it1 == list.end ()) return;
      std::list <BodyPairCollisionPtr_t>::iterator it2 = it1; ++it2;
      while (it2 != list.end ()) {
	if (comp (*it2, *it1))
	  std::iter_swap (it1, it2);
	else
	  return;
	it1 = it2;
	++it2;
      }      
    }
    ContinuousCollisionCheckingPtr_t
    ContinuousCollisionChecking::create (const DevicePtr_t& robot,
					 const value_type& tolerance)
    {
      ContinuousCollisionChecking* ptr =
	new ContinuousCollisionChecking (robot, tolerance);
      ContinuousCollisionCheckingPtr_t shPtr (ptr);
      return shPtr;
    }

    bool ContinuousCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      StraightPathPtr_t straightPath = HPP_DYNAMIC_PTR_CAST
	(StraightPath, path);
      // for each BodyPairCollision
      //   - set path,
      //   - compute valid interval at start (end if reverse)
      value_type t0 = path->timeRange ().first;
      value_type t1 = path->timeRange ().second;
      if (reverse) {
	for (std::list <BodyPairCollisionPtr_t>::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  (*itPair)->path (straightPath);
	  // If collision at end point, return false
	  if (!(*itPair)->validateInterval (t1)) {
	    validPart = path->extract (interval_t (t1, t1));
	    return false;
	  }
	  assert ((*itPair)->validSubset ().contains (t1));
	}
	// Sort collision pairs
	bodyPairCollisions_.sort (compareReverseBodyPairCol);

	BodyPairCollisionPtr_t first = *(bodyPairCollisions_.begin ());
	while (!first->validSubset ().contains (t0, true)) {
	  // find middle of first non valid interval
	  const std::list <interval_t>& intervals =
	    first->validSubset ().list ();
	  std::list <interval_t>::const_reverse_iterator lastInterval =
	    intervals.rbegin ();
	  std::list <interval_t>::const_reverse_iterator beforeLastInterval =
	    lastInterval; ++beforeLastInterval;
	  value_type upper = lastInterval->first;
	  value_type lower;
	  if (beforeLastInterval != intervals.rend ()) {
	    lower = beforeLastInterval->second;
	  } else {
	    lower = t0;
	  }
	  value_type middle = .5 * (lower + upper);
	  if (first->validateInterval (middle)) {
	    partialSort (bodyPairCollisions_, compareReverseBodyPairCol);
	  } else {
	    upper = computeLastValidParameter (bodyPairCollisions_, true);
	    validPart = path->extract (interval_t (upper, t1));
	    return false;
	  }
	  first = *(bodyPairCollisions_.begin ());
	}
      } else {
	for (std::list <BodyPairCollisionPtr_t>::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  (*itPair)->path (straightPath);
	  // If collision at start point, return false
	  bool valid = (*itPair)->validateInterval (t0);
	  if (!valid) {
	    validPart = path->extract (interval_t (t0, t0));
	    return false;
	  }
	  assert ((*itPair)->validSubset ().contains (t0));
	}
	// Sort collision pairs
	bodyPairCollisions_.sort (compareBodyPairCol);
	BodyPairCollisionPtr_t first = *(bodyPairCollisions_.begin ());
	while (!first->validSubset ().contains (t1)) {
	  // find middle of first non valid interval
	  const std::list <interval_t>& intervals =
	    first->validSubset ().list ();
	  std::list <interval_t>::const_iterator firstInterval =
	    intervals.begin ();
	  std::list <interval_t>::const_iterator secondInterval = firstInterval;
	  ++secondInterval;
	  value_type lower = firstInterval->second;
	  value_type upper;
	  if (secondInterval != intervals.end ()) {
	    upper = secondInterval->first;
	  } else {
	    upper = t1;
	  }
	  value_type middle = .5 * (lower + upper);
	  if (first->validateInterval (middle)) {
	    partialSort (bodyPairCollisions_, compareBodyPairCol);
	  } else {
	    lower = computeLastValidParameter (bodyPairCollisions_, false);
	    validPart = path->extract (interval_t (t0, lower));
	    return false;
	  }
	  first = *(bodyPairCollisions_.begin ());
	}
      }
      validPart = path;
      return true;
    }

    void ContinuousCollisionChecking::addObstacle
    (const CollisionObjectPtr_t& object)
    {
      for (std::list <BodyPairCollisionPtr_t>::iterator itPair =
	     bodyPairCollisions_.begin ();
	   itPair != bodyPairCollisions_.end (); ++itPair) {
	if (!(*itPair)->joint_b ()) {
	  (*itPair)->addObjectTo_b (object);
	}
      }
    }

    ContinuousCollisionChecking::~ContinuousCollisionChecking ()
    {
    }

    ContinuousCollisionChecking::ContinuousCollisionChecking
    (const DevicePtr_t& robot, const value_type& tolerance) :
      robot_ (robot), tolerance_ (tolerance),
      bodyPairCollisions_ ()
    {
      typedef model::Device::CollisionPairs_t CollisionPairs_t;
      using continuousCollisionChecking::BodyPairCollision;
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
      // For each joint, extract from the body the list of obstacles that
      // not attached to a joint
      ObjectVector_t objects;
      for (JointVector_t::const_iterator itJoint =
	     robot->getJointVector ().begin ();
	   itJoint != robot->getJointVector ().end (); ++itJoint) {
	BodyPtr_t body = (*itJoint)->linkedBody ();
	if (body) {
	  objects.clear ();
	  for (ObjectVector_t::const_iterator itObj =
		 body->outerObjects (model::COLLISION).begin ();
	       itObj != body->outerObjects (model::COLLISION).end (); ++itObj) {
	    if (!(*itObj)->joint () || (*itObj)->joint ()->robot () != robot) {
	      objects.push_back (*itObj);
	    }
	  }
	  bodyPairCollisions_.push_back (BodyPairCollision::create
					 (*itJoint, objects, tolerance_));
	}
      }
    }
  } // namespace core
} // namespace hpp
