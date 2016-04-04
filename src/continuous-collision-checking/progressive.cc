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
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/continuous-collision-checking/progressive.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-collision-checking/progressive/body-pair-collision.hh"

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {

      using progressive::BodyPairCollision;
      using progressive::BodyPairCollisionPtr_t;
      using progressive::BodyPairCollisions_t;

      ProgressivePtr_t Progressive::create (const DevicePtr_t& robot,
					    const value_type& tolerance)
      {
	Progressive* ptr =
	  new Progressive (robot, tolerance);
	ProgressivePtr_t shPtr (ptr);
	return shPtr;
      }

      bool Progressive::validateConfiguration
      (const Configuration_t& config, bool reverse, value_type& tmin,
       PathValidationReportPtr_t& report)
      {
	value_type t = tmin;
	tmin = std::numeric_limits <value_type>::infinity ();
	value_type tmpMin;
	robot_->currentConfiguration (config);
	robot_->computeForwardKinematics ();
	for (BodyPairCollisions_t::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  CollisionValidationReportPtr_t collisionReport;
	  if (!(*itPair)->validateConfiguration (t, tmpMin, collisionReport)) {
	    report = CollisionPathValidationReportPtr_t
	      (new CollisionPathValidationReport);
	    report->configurationReport = collisionReport;
	    report->parameter = t;
	    return false;
	  } else {
	    if (reverse) {
	      tmin = std::max (tmin, tmpMin);
	    } else {
	      tmin = std::min (tmin, tmpMin);
	    }
	  }
	}
	hppDout (info, "tmin=" << tmin);
	return true;
      }

      bool Progressive::validate (const PathPtr_t& path, bool reverse,
				  PathPtr_t& validPart,
				  PathValidationReportPtr_t& report)
      {
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
	// for each BodyPairCollision
	//   - set path,
	//   - compute valid interval at start (end if reverse)
	bool valid = true;
	for (BodyPairCollisions_t::iterator itPair =
	       bodyPairCollisions_.begin ();
	     itPair != bodyPairCollisions_.end (); ++itPair) {
	  (*itPair)->path (path, reverse);
	}
	if (reverse) {
	  value_type tmin = path->timeRange ().first;
	  value_type tmax = path->timeRange ().second;
	  value_type lastValidTime = tmax;
	  value_type t = tmax;
	  unsigned finished = 0;
          Configuration_t q (path->outputSize ());
	  while (finished < 2 && valid) {
	    bool success = (*path) (q, t);
	    value_type tprev = t;
	    PathValidationReportPtr_t pathReport;
	    if (!success || !validateConfiguration (q, reverse, t, pathReport)) {
	      report = pathReport;
	      valid = false;
	    } else {
	      lastValidTime = tprev;
	    }
	    if (t <= tmin) {
	      t = tmin;
	      finished ++;
	    }
	  }
	  if (valid) {
	    validPart = path;
	    return true;
	  } else {
	    validPart = path->extract (std::make_pair (lastValidTime, tmax));
	    return false;
	  }
	} else {
	  value_type tmin = path->timeRange ().first;
	  value_type tmax = path->timeRange ().second;
	  value_type lastValidTime = tmin;
	  value_type t = tmin;
	  unsigned finished = 0;
          Configuration_t q (path->outputSize ());
          if (t >= tmax) finished++;
	  while (finished < 2 && valid) {
	    bool success = (*path) (q, t);
	    value_type tprev = t;
	    PathValidationReportPtr_t pathReport;
	    if (!success || !validateConfiguration (q, reverse, t, pathReport)) {
	      report = pathReport;
	      valid = false;
	    } else {
	      lastValidTime = tprev;
	    }
	    if (t >= tmax) {
	      t = tmax;
	      finished ++;
	    }
	  }
	  if (valid) {
	    validPart = path;
	    return true;
	  } else {
	    validPart = path->extract (std::make_pair (tmin, lastValidTime));
	    return false;
	  }
	}
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
	  oss << "Progressive::removeObstacleFromJoint: obstacle \""
	      << obstacle->name () <<
	    "\" is not registered as obstacle for joint \"" << joint->name ()
	      << "\".";
	  throw std::runtime_error (oss.str ());
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
