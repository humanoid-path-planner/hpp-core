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
#include <hpp/core/collision-validation.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/relative-motion.hh>

namespace hpp {
  namespace core {
    namespace {
      inline std::size_t collide (const CollisionPairs_t::const_iterator& _colPair,
          const fcl::CollisionRequest& req, fcl::CollisionResult& res) {
        return fcl::collide (
                    _colPair->first ->fcl ().get (),
                    _colPair->second->fcl ().get (),
                    req, res);
      }

      inline bool collide (const CollisionPairs_t& pairs,
         const fcl::CollisionRequest& req, fcl::CollisionResult& res,
         CollisionPairs_t::const_iterator& _col) {
        for (_col = pairs.begin (); _col != pairs.end (); ++_col)
          if (collide (_col, req, res) != 0)
            return true;
        return false;
      }
    }
    using model::displayConfig;

    typedef model::JointConfiguration* JointConfigurationPtr_t;
    CollisionValidationPtr_t CollisionValidation::create
    (const DevicePtr_t& robot)
    {
      CollisionValidation* ptr = new CollisionValidation (robot);
      return CollisionValidationPtr_t (ptr);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
					ValidationReportPtr_t& validationReport)
    {
      robot_->currentConfiguration (config);
      robot_->computeForwardKinematics ();
      fcl::CollisionResult collisionResult;
      CollisionPairs_t::const_iterator _col;
      if (collide (collisionPairs_, collisionRequest_, collisionResult, _col)
          ||
          ( checkParameterized_ &&
            collide (parameterizedPairs_, collisionRequest_, collisionResult, _col)
          )) {
        CollisionValidationReportPtr_t report (new CollisionValidationReport);
        report->object1 = _col->first;
        report->object2 = _col->second;
        report->result = collisionResult;
        validationReport = report;
        return false;
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

    void CollisionValidation::filterCollisionPairs (const RelativeMotion::matrix_type& matrix)
    {
      // Loop over collision pairs and remove disabled ones.
      CollisionPairs_t::iterator _colPair = collisionPairs_.begin ();
      size_type i1, i2;
      fcl::CollisionResult unused;
      while (_colPair != collisionPairs_.end ()) {
        const JointPtr_t& j1 = _colPair->first ->joint(),
                          j2 = _colPair->second->joint();
        i1 = RelativeMotion::idx(j1);
        i2 = RelativeMotion::idx(j2);
        switch (matrix(i1, i2)) {
          case RelativeMotion::Parameterized:
              hppDout(info, "Parameterized collision pairs between "
                  << _colPair->first ->name() << " and "
                  << _colPair->second->name());
              parameterizedPairs_.push_back (*_colPair);
              _colPair = collisionPairs_.erase (_colPair);
              break;
          case RelativeMotion::Constrained:
              hppDout(info, "Disabling collision between "
                  << _colPair->first ->name() << " and "
                  << _colPair->second->name());
              if (collide (_colPair, collisionRequest_, unused) != 0) {
                hppDout(warning, "Disabling collision detection between two "
                    "body in collision.");
              }
              disabledPairs_.push_back (*_colPair);
              _colPair = collisionPairs_.erase (_colPair);
              break;
          case RelativeMotion::Unconstrained: ++_colPair; break;
          default:
            hppDout (warning, "RelativeMotionType not understood");
            ++_colPair;
            break;
        }
      }
    }

    void CollisionValidation::randomnizeCollisionPairs(){
      std::vector<CollisionPair_t> v;
      v.reserve(collisionPairs_.size());
      v.insert(v.end(),collisionPairs_.begin(),collisionPairs_.end());
      std::random_shuffle(v.begin(), v.end());
      collisionPairs_.clear();
      collisionPairs_.insert(collisionPairs_.end(),v.begin(),v.end());
    }


    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      collisionRequest_(1, false, false, 1, false, true, fcl::GST_INDEP),
      robot_ (robot),
      parameterizedPairs_(), disabledPairs_(),
      checkParameterized_(false)
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
