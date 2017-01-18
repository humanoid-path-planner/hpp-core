//
// Copyright (c) 2014, 2015, 2016, 2017 CNRS
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

#include <hpp/core/continuous-collision-checking.hh>

#include <limits>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-collision-checking/body-pair-collision.hh"

namespace hpp {
  namespace core {
    namespace {
      typedef std::pair<se3::JointIndex, se3::JointIndex> JointIndexPair_t;

      struct JointIndexPairCompare_t {
	bool operator() (const JointIndexPair_t& p0, const JointIndexPair_t& p1) const
	{
	  if (p0.first < p1.first) return true;
	  if (p0.first > p1.first) return false;
	  return (p0.second < p1.second);
	}
      };

      using continuousCollisionChecking::BodyPairCollision;
      using continuousCollisionChecking::BodyPairCollisions_t;
      using continuousCollisionChecking::BodyPairCollisionPtr_t;

      typedef std::set<JointIndexPair_t, JointIndexPairCompare_t> JointIndexPairSet_t;
    }

    /// Validate interval centered on a path parameter
    /// \param config Configuration at abscissa tmin on the path.
    /// \param tmin parameter value in the path interval of definition
    /// \retval tmin lower bound of valid interval if reverve is true,
    ///              upper bound of valid interval if reverse is false.
    ///              other bound of valid interval is input tmin.
    /// \return true if the configuration is collision free for this parameter
    ///         value, false otherwise.
    bool ContinuousCollisionChecking::validateConfiguration
    (const Configuration_t& config, const value_type& t,
     interval_t& interval, PathValidationReportPtr_t& report)
    {
      interval.first = -std::numeric_limits <value_type>::infinity ();
      interval.second = std::numeric_limits <value_type>::infinity ();
      interval_t tmpInt;
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics();
      robot_->updateGeometryPlacements();
      for (BodyPairCollisions_t::iterator itPair =
	     bodyPairCollisions_.begin ();
	   itPair != bodyPairCollisions_.end (); ++itPair) {
	CollisionValidationReportPtr_t collisionReport;
	if (!(*itPair)->validateConfiguration (t, tmpInt, collisionReport)) {
	  report = CollisionPathValidationReportPtr_t
	    (new CollisionPathValidationReport);
	  report->configurationReport = collisionReport;
	  report->parameter = t;
	  return false;
	} else {
	  interval.first = std::max (interval.first, tmpInt.first);
	  interval.second = std::min (interval.second, tmpInt.second);
	  assert (interval.second > interval.first);
	  assert (interval.first <= t);
	  assert (t <= interval.second);
	}
      }
      return true;
    }

    bool ContinuousCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
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
      return validateStraightPath (path, reverse, validPart, report);
    }

    void ContinuousCollisionChecking::addObstacle
    (const CollisionObjectConstPtr_t& object)
    {
      pinocchio::JointVector_t& jv = robot_->getJointVector ();
      for (size_type idx = 0; idx < jv.size (); ++idx) {
	BodyPtr_t body = (jv.at(idx))->linkedBody ();
	if (body) {
	  ConstObjectStdVector_t objects;
	  objects.push_back (object);
	  bodyPairCollisions_.push_back
	    (BodyPairCollision::create (jv.at(idx), objects, tolerance_));
	  hppDout(info, *bodyPairCollisions_.back());
	}
      }
    }

    void ContinuousCollisionChecking::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle)
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
	oss << "ContinuousCollisionChecking::removeObstacleFromJoint: obstacle \""
	    << obstacle->name () <<
	  "\" is not registered as obstacle for joint \"" << joint->name ()
	    << "\".";
	throw std::runtime_error (oss.str ());
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
      // Build body pairs for collision checking
      // First auto-collision
      // FIXME The pairs of joint are duplicated
      const se3::GeometryModel& gmodel = robot->geomModel ();
      JointPtr_t joint1, joint2;
      JointIndexPairSet_t jointPairs;
      std::size_t duplicates = 0;
      for (std::size_t i = 0; i < gmodel.collisionPairs.size(); ++i)
        {
          const se3::CollisionPair& cp = gmodel.collisionPairs[i];
          JointIndexPair_t jp (
			       gmodel.geometryObjects[cp.first].parentJoint,
			       gmodel.geometryObjects[cp.second].parentJoint);
          if (jointPairs.count(jp) == 0) {
            jointPairs.insert(jp);
            joint1 = JointPtr_t(new Joint(robot_, jp.first));
            joint2 = JointPtr_t(new Joint(robot_, jp.second));
            bodyPairCollisions_.push_back (BodyPairCollision::create
					   (joint2, joint1, tolerance_));
            hppDout(info, *bodyPairCollisions_.back());
          } else duplicates++;
	}
    }
  } // namespace core
} // namespace hpp
