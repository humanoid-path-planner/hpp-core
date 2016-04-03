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

#include <hpp/fcl/collision.h>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    typedef model::JointConfiguration* JointConfigurationPtr_t;
    CollisionValidationPtr_t CollisionValidation::create
    (const DevicePtr_t& robot)
    {
      CollisionValidation* ptr = new CollisionValidation (robot);
      return CollisionValidationPtr_t (ptr);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					bool throwIfInValid)
    {
      return validate (config, unusedReport, throwIfInValid);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					ValidationReport& validationReport,
					bool throwIfInValid)
    {
      HPP_STATIC_CAST_REF_CHECK (CollisionValidationReport, validationReport);
      CollisionValidationReport& report =
	static_cast <CollisionValidationReport&> (validationReport);
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics ();
      bool collision = false;
      fcl::CollisionResult& collisionResult = report.result;
      collisionResult.clear();
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
	if (fcl::collide (itCol->first->fcl ().get (),
			  itCol->second->fcl ().get (),
			  collisionRequest_, collisionResult) != 0) {
	  report.object1 = itCol->first;
	  report.object2 = itCol->second;
	  collision = true;
	  break;
	}
      }
      if (collision && throwIfInValid) {
	std::ostringstream oss ("Configuration in collision: ");
	oss << displayConfig (config);
	throw std::runtime_error (oss.str ());
      }
      return !collision;
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					ValidationReportPtr_t& validationReport)
    {
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics ();
      fcl::CollisionResult collisionResult;
      for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
	   itCol != collisionPairs_.end (); ++itCol) {
	if (fcl::collide (itCol->first->fcl ().get (),
			  itCol->second->fcl ().get (),
			  collisionRequest_, collisionResult) != 0) {
	  CollisionValidationReportPtr_t report (new CollisionValidationReport);
	  report->object1 = itCol->first;
	  report->object2 = itCol->second;
	  report->result = collisionResult;
	  validationReport = report;
	  return false;
	}
      }
      return true;
    }


    void CollisionValidation::addObstacle (const CollisionObjectPtr_t& object)
    {
      using model::COLLISION;
      const JointVector_t& jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
	   ++it) {
	JointPtr_t joint = *it;
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
	  const ObjectVector_t& bodyObjects = body->innerObjects (COLLISION);
	  for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	       itInner != bodyObjects.end (); ++itInner) {
	    collisionPairs_.push_back (CollisionPair_t (*itInner, object));
	  }
	}
      }
    }

    void CollisionValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
      using model::COLLISION;
      BodyPtr_t body = joint->linkedBody ();
      if (body) {
	const ObjectVector_t& bodyObjects = body->innerObjects (COLLISION);
	for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	     itInner != bodyObjects.end (); ++itInner) {
	  CollisionPair_t colPair (*itInner, obstacle);
	  std::size_t before = collisionPairs_.size ();
	  collisionPairs_.remove (colPair);
	  std::size_t after = collisionPairs_.size ();
	  if (after == before) {
	    std::ostringstream oss;
	    oss << "CollisionValidation::removeObstacleFromJoint: obstacle \""
		<< obstacle->name () <<
	      "\" is not registered as obstacle for joint \"" << joint->name ()
		<< "\".";
	    throw std::runtime_error (oss.str ());
	  } else if (before - after >= 2) {
	    hppDout (error, "obstacle "<< obstacle->name () <<
		     " was registered " << before - after
		     << " times as obstacle for joint " << joint->name ()
		     << ".");
	  }
	}
      }
    }

    size_type CollisionValidation::filterCollisionPairs (const ConstraintSetPtr_t& c)
    {
      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return 0;
      const LockedJoints_t& lj = proj->lockedJoints ();
      // true
      typedef std::map <std::string, std::size_t> StringIndex_m;
      typedef std::list<JointPtr_t> Joints_t;
      typedef std::vector<Joints_t> JointLists_t;

      StringIndex_m nameToIndex;
      // A vector of (list of sequential locked joints)
      JointLists_t jls;
      for (LockedJoints_t::const_iterator it = lj.begin ();
          it != lj.end (); ++it) {
        // Extra dofs and partial locked joints
        JointPtr_t j = robot_->getJointByName ((*it)->jointName());
        JointPtr_t parent = j->parentJoint ();
        while (parent != NULL && parent->numberDof() == 0)
          parent = parent->parentJoint();
        if (parent == NULL ||
             nameToIndex.find (parent->name()) == nameToIndex.end()) {
          // Create a new list
          nameToIndex[j->name()] = jls.size();
          Joints_t js;
          if (parent == NULL) js.push_back (NULL);
          js.push_back (j);
          jls.push_back (js);
        } else {
          std::size_t i = nameToIndex[parent->name()];
          jls[i].push_back (j);
          nameToIndex[j->name()] = i;
        }
      }
      // Build the matrix of joint pairs to be disabled.
      Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> disable
        (robot_->numberDof() + 1, robot_->numberDof() + 1);
      size_type i1, i2;
      disable.setConstant (false);
      for (JointLists_t::const_iterator _jls = jls.begin();
          _jls != jls.end(); ++_jls) {
        for (Joints_t::const_iterator _js1 = _jls->begin();
            _js1 != _jls->end(); ++_js1) {
          for (Joints_t::const_iterator _js2 = _js1;
              _js2 != _jls->end(); ++_js2) {
            i1 = ((*_js1 == NULL) ? robot_->numberDof () : (*_js1)->rankInVelocity());
            i2 = ((*_js2 == NULL) ? robot_->numberDof () : (*_js2)->rankInVelocity());
            disable(i1, i2) = true;
            disable(i2, i1) = true;
          }
        }
      }
      // Loop over collision pairs and remove disabled ones.
      CollisionPairs_t::iterator _colPairs = collisionPairs_.begin ();
      size_type ret = 0;
      while (_colPairs != collisionPairs_.end ()) {
        i1 = (_colPairs->first ->joint() == NULL) ? robot_->numberDof () : _colPairs->first ->joint()->rankInVelocity();
        i2 = (_colPairs->second->joint() == NULL) ? robot_->numberDof () : _colPairs->second->joint()->rankInVelocity();
        if (disable (i1, i2)) {
          CollisionPair_t pair = *_colPairs;
          _colPairs = collisionPairs_.erase (_colPairs);
          disabledPairs_.push_back (pair);
          ++ret;
          hppDout(info, "Disabling collision between "
              << _colPairs->first ->name() << " and "
              << _colPairs->second->name());
        } else ++_colPairs;
      }
      return ret;
    }

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      collisionRequest_(1, false, false, 1, false, true, fcl::GST_INDEP),
      robot_ (robot),
      disabledPairs_ ()
    {
      using model::COLLISION;
      typedef hpp::model::Device::CollisionPairs_t JointPairs_t;
      using model::ObjectVector_t;
      const JointPairs_t& jointPairs (robot->collisionPairs (COLLISION));
      // build collision pairs for internal objects
      for (JointPairs_t::const_iterator it = jointPairs.begin ();
	   it != jointPairs.end (); ++it) {
	JointPtr_t j1 = it->first;
	JointPtr_t j2 = it->second;
	BodyPtr_t body1 = j1->linkedBody ();
	BodyPtr_t body2 = j2->linkedBody ();
	if (body1 && body2) {
	  ObjectVector_t objects1 = body1->innerObjects (COLLISION);
	  ObjectVector_t objects2 = body2->innerObjects (COLLISION);
	  // Loop over pairs of inner objects of the bodies
	  for (ObjectVector_t::const_iterator it1 = objects1.begin ();
	       it1 != objects1.end (); ++it1) {
	    for (ObjectVector_t::const_iterator it2 = objects2.begin ();
		 it2 != objects2.end (); ++it2) {
	      collisionPairs_.push_back (CollisionPair_t (*it1, *it2));
	    }
	  }
	}

      }
    }
  } // namespace core
} // namespace hpp
