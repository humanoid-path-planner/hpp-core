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
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/configuration.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/relative-motion.hh>

namespace hpp {
  namespace core {

    using pinocchio::displayConfig;

    typedef pinocchio::JointConfiguration* JointConfigurationPtr_t;
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
      se3::GeometryData::CollisionPairsVector_t::const_iterator _col;
      bool collision = false;
      for (_col = geomData_->collision_pairs.begin ();
              _col != geomData_->collision_pairs.end (); ++_col) {
          collisionResult = (geomData_->computeCollision (_col->first,
                  _col->second)).fcl_collision_result;
          if (collisionResult.isCollision ()){
              collision = true;
              break;
          }
      } 
      // if no collision found and parameterised objects should be checked,
      // go through them looking for collisions
      if (!collision && checkParameterized_) {
          for (_col = parameterizedPairs_.begin ();
              _col != parameterizedPairs_.end (); ++_col) {
              collisionResult = (geomData_->computeCollision (_col->first,
                  _col->second)).fcl_collision_result;
              if (collisionResult.isCollision ()){
                  collision = true;
                  break;
              }
          }
       }
       if (collision) {
          CollisionValidationReportPtr_t report (new CollisionValidationReport);
          report->object1 = CollisionObjectPtr_t (new pinocchio::CollisionObject (robot_,
                      robot_->geomModel ().geometryObjects[_col->first].parent,
                      _col->first));
          report->object2 = CollisionObjectPtr_t (new pinocchio::CollisionObject (robot_,
                      robot_->geomModel ().geometryObjects[_col->second].parent,
                      _col->second));
          report->result = collisionResult;
          validationReport = report;
          return false;
      }
      return true;
    }

    void CollisionValidation::addObstacle (const CollisionObjectPtr_t& object)
    {
      using pinocchio::COLLISION;
      const JointVector_t& jv = robot_->getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
	   ++it) {
	JointPtr_t joint = JointPtr_t (new Joint(**it));
	BodyPtr_t body = joint->linkedBody ();
	if (body) {
	  const ObjectVector_t& bodyObjects = body->innerObjects ();
	  for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	       itInner != bodyObjects.end (); ++itInner) {
        // TODO: check the objects are not in same joint
      geomData_->addCollisionPair ((*itInner)->indexInModel (), object->indexInModel ());
	  }
	}
      }
    }

    void CollisionValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle)
    {
      using pinocchio::COLLISION;
      BodyPtr_t body = joint->linkedBody ();
      if (body) {
	const ObjectVector_t& bodyObjects = body->innerObjects ();
	for (ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
	     itInner != bodyObjects.end (); ++itInner) {
	  CollisionPair_t colPair = std::make_pair(*itInner, obstacle);
	  std::size_t before = geomData_->nCollisionPairs;
	  geomData_->removeCollisionPair (colPair.first->indexInModel (),
            colPair.second->indexInModel ());
	  std::size_t after = geomData_->nCollisionPairs;
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
      se3::GeometryData::CollisionPairsVector_t::iterator _colPair = geomData_->collision_pairs.begin ();
      se3::GeometryObject::JointIndex i1, i2;
      fcl::CollisionResult res;
      while (_colPair != geomData_->collision_pairs.end ()) {
        i1 = robot_->geomModel ().geometryObjects[_colPair->first].parent;
        i2 = robot_->geomModel ().geometryObjects[_colPair->second].parent;
        switch (matrix(i1, i2)) {
          case RelativeMotion::Parameterized:
              hppDout(info, "Parameterized collision pairs between "
                  << robot_->geomModel ().getGeometryName(_colPair->first) << " and "
                  << robot_->geomModel ().getGeometryName(_colPair->second));
              parameterizedPairs_.push_back (*_colPair);
              geomData_->removeCollisionPair (_colPair->first,
                      _colPair->second);
              // previously _colPair = deleteCollisionPair... How to compensate?
              break;
          case RelativeMotion::Constrained:
              hppDout(info, "Disabling collision between "
                  << robot_->geomModel ().getGeometryName(_colPair->first) << " and "
                  << robot_->geomModel ().getGeometryName(_colPair->second));
              res = geomData_->computeCollision (_colPair->first, _colPair->second).fcl_collision_result;
              if (res.isCollision ()) {
                hppDout(warning, "Disabling collision detection between two "
                    "bodies in collision.");
              }
              disabledPairs_.push_back (*_colPair);
              geomData_->removeCollisionPair (_colPair->first, _colPair->second);
              // previously _colPair = deleteCollisionPair... How to compensate?
              break;
          case RelativeMotion::Unconstrained: ++_colPair; break;
          default:
            hppDout (warning, "RelativeMotionType not understood");
            ++_colPair;
            break;
        }
      }
    }

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      robot_ (robot),
      geomData_ (robot->geomData ()),
      parameterizedPairs_(), disabledPairs_(),
      checkParameterized_(false)
          {}

  } // namespace core
} // namespace hpp
