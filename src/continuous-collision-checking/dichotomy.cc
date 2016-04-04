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

#include <deque>
#include <hpp/util/debug.hh>
#include <hpp/core/continuous-collision-checking/dichotomy.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-collision-checking/dichotomy/body-pair-collision.hh"

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {

      using dichotomy::BodyPairCollision;
      using dichotomy::BodyPairCollisionPtr_t;
      using dichotomy::BodyPairCollisions_t;

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

      // Partially sort a list where only the first element is not sorted
      template <typename Compare>
      void partialSort (BodyPairCollisions_t& list, Compare comp)
      {
	BodyPairCollisions_t::iterator it1 = list.begin ();
	if (it1 == list.end ()) return;
	BodyPairCollisions_t::iterator it2 = it1; ++it2;
	while (it2 != list.end ()) {
	  if (comp (*it2, *it1))
	    std::iter_swap (it1, it2);
	  else
	    return;
	  it1 = it2;
	  ++it2;
	}
      }
      DichotomyPtr_t
      Dichotomy::create (const DevicePtr_t& robot, const value_type& tolerance)
      {
	Dichotomy* ptr =
	  new Dichotomy (robot, tolerance);
	DichotomyPtr_t shPtr (ptr);
	return shPtr;
      }

      bool Dichotomy::validate (const PathPtr_t& path, bool reverse,
				PathPtr_t& validPart,
				PathValidationReportPtr_t& report)
      {
	CollisionValidationReportPtr_t collisionReport
	  (new CollisionValidationReport);
	if (PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, path)) {
	  PathVectorPtr_t validPathVector = PathVector::create
	    (path->outputSize (), path->outputDerivativeSize ());
	  validPart = validPathVector;
	  PathPtr_t localValidPart;
	  if (reverse) {
	    value_type param = path->length ();
	    std::deque <PathPtr_t> paths;
	    for (std::size_t i=pv->numberPaths () + 1; i != 0 ; --i) {
	      PathPtr_t localPath (pv->pathAtRank (i-1));
	      if (validate (localPath, reverse, localValidPart, report)) {
		paths.push_front (localPath->copy ());
		param -= localPath->length ();
	      } else {
		report->parameter += param - localPath->length ();
		paths.push_front (localValidPart->copy ());
		for (std::deque <PathPtr_t>::const_iterator it = paths.begin ();
		     it != paths.end (); ++it) {
		  validPathVector->appendPath (*it);
		}
		return false;
	      }
	    }
	    return true;
	  } else {
	    value_type param = 0;
	    for (std::size_t i=0; i < pv->numberPaths (); ++i) {
	      PathPtr_t localPath (pv->pathAtRank (i));
	      if (validate (localPath, reverse, localValidPart, report)) {
		validPathVector->appendPath (localPath->copy ());
		param += localPath->length ();
	      } else {
		report->parameter += param;
		validPathVector->appendPath (localValidPart->copy ());
		return false;
	      }
	    }
	    return true;
	  }
	}
	StraightPathPtr_t straightPath = HPP_DYNAMIC_PTR_CAST
	  (StraightPath, path);
	// for each BodyPairCollision
	//   - set path,
	//   - compute valid interval at start (end if reverse)
	value_type t0 = path->timeRange ().first;
	value_type t1 = path->timeRange ().second;
	if (reverse) {
	  for (BodyPairCollisions_t::iterator itPair =
		 bodyPairCollisions_.begin ();
	       itPair != bodyPairCollisions_.end (); ++itPair) {
	    (*itPair)->path (straightPath);
	    // If collision at end point, return false
	    if (!(*itPair)->validateInterval (t1, *collisionReport)) {
	      report = CollisionPathValidationReportPtr_t
		(new CollisionPathValidationReport (t1, collisionReport));
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
	    if (first->validateInterval (middle, *collisionReport)) {
	      partialSort (bodyPairCollisions_, compareReverseBodyPairCol);
	    } else {
	      report = CollisionPathValidationReportPtr_t
		(new CollisionPathValidationReport (middle, collisionReport));
	      validPart = path->extract (interval_t (upper, t1));
	      return false;
	    }
	    first = *(bodyPairCollisions_.begin ());
	  }
	} else {
	  for (BodyPairCollisions_t::iterator itPair =
		 bodyPairCollisions_.begin ();
	       itPair != bodyPairCollisions_.end (); ++itPair) {
	    (*itPair)->path (straightPath);
	    // If collision at start point, return false
	    bool valid = (*itPair)->validateInterval (t0, *collisionReport);
	    if (!valid) {
	      report = CollisionPathValidationReportPtr_t
		(new CollisionPathValidationReport (t0, collisionReport));
	      validPart = path->extract (interval_t (t0, t0));
	      hppDout (error, "Initial position in collision.");
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
	    std::list <interval_t>::const_iterator secondInterval =
	      firstInterval;
	    ++secondInterval;
	    value_type lower = firstInterval->second;
	    value_type upper;
	    if (secondInterval != intervals.end ()) {
	      upper = secondInterval->first;
	    } else {
	      upper = t1;
	    }
	    value_type middle = .5 * (lower + upper);
	    if (first->validateInterval (middle, *collisionReport)) {
	      partialSort (bodyPairCollisions_, compareBodyPairCol);
	    } else {
	      report = CollisionPathValidationReportPtr_t
		(new CollisionPathValidationReport (middle, collisionReport));
	      validPart = path->extract (interval_t (t0, lower));
	      return false;
	    }
	    first = *(bodyPairCollisions_.begin ());
	  }
	}
	validPart = path;
	return true;
      }

      void Dichotomy::addObstacle
      (const CollisionObjectPtr_t& object)
      {
	const JointVector_t& jv = robot_->getJointVector ();
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); ++itJoint) {
	  BodyPtr_t body = (*itJoint)->linkedBody ();
	  bool foundPair = false;
	  if (body) {
	    for (BodyPairCollisions_t::iterator itPair =
		   bodyPairCollisions_.begin ();
		 itPair != bodyPairCollisions_.end (); ++itPair) {
	      if (((*itPair)->joint_a () == *itJoint) &&
		  (!(*itPair)->joint_b ())) {
		(*itPair)->addObjectTo_b (object);
		foundPair = true;
	      }
	    }
	    if (!foundPair) {
	      ObjectVector_t objects;
	      objects.push_back (object);
	      bodyPairCollisions_.push_back
		(BodyPairCollision::create (*itJoint, objects, tolerance_));
	    }
	  }
	}
      }

      void Dichotomy::removeObstacleFromJoint
      (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
      {
	bool removed = false;
	for (BodyPairCollisions_t::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  if (!(*itPair)->joint_b () && (*itPair)->joint_a () == joint) {
	    if ((*itPair)->removeObjectTo_b (obstacle)) {
	      removed = true;
	      if ((*itPair)->objects_b ().empty ()) {
		bodyPairCollisions_.erase (itPair);
	      }
	    }
	  }
	}
	if (!removed) {
	  std::ostringstream oss;
	  oss << "Dichotomy::removeObstacleFromJoint: obstacle \""
	      << obstacle->name () <<
	    "\" is not registered as obstacle for joint \"" << joint->name ()
	      << "\".";
	  throw std::runtime_error (oss.str ());
	}
      }

      Dichotomy::~Dichotomy ()
      {
      }

      Dichotomy::Dichotomy
      (const DevicePtr_t& robot, const value_type& tolerance) :
	robot_ (robot), tolerance_ (tolerance),
	bodyPairCollisions_ ()
      {
	// Tolerance should be equal to 0, otherwise end of valid
	// sub-path might be in collision.
	if (tolerance != 0) {
	  throw std::runtime_error ("Dichotomy path validation method does not"
				    "support penetration.");
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
