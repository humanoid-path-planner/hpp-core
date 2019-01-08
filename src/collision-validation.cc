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

#include <hpp/core/collision-validation.hh>

#include <hpp/fcl/collision.h>

#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/relative-motion.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/relative-motion.hh>

namespace hpp {
  namespace core {
    namespace {
      inline std::size_t collide (const CollisionPairs_t::const_iterator& _colPair,
          const fcl::CollisionRequest& req, fcl::CollisionResult& res,
          pinocchio::DeviceData& data) {
        res.clear();
        return fcl::collide (
                    _colPair->first ->fcl (data),
                    _colPair->second->fcl (data),
                    req, res);
      }

      inline bool collide (const CollisionPairs_t& pairs,
         const fcl::CollisionRequest& req, fcl::CollisionResult& res,
         CollisionPairs_t::const_iterator& _col,
         pinocchio::DeviceData& data) {
        for (_col = pairs.begin (); _col != pairs.end (); ++_col)
          if (collide (_col, req, res, data) != 0)
            return true;
        return false;
      }

      inline CollisionPair_t makeCollisionPair (const DevicePtr_t& d, const se3::CollisionPair& p)
      {
        CollisionObjectConstPtr_t o1 (new pinocchio::CollisionObject(d, p.first));
        CollisionObjectConstPtr_t o2 (new pinocchio::CollisionObject(d, p.second));
        return CollisionPair_t (o1, o2);
      }
    }
    CollisionValidationPtr_t CollisionValidation::create
    (const DevicePtr_t& robot)
    {
      CollisionValidation* ptr = new CollisionValidation (robot);
      return CollisionValidationPtr_t (ptr);
    }

    bool CollisionValidation::validate (const Configuration_t& config,
                                        ValidationReportPtr_t& validationReport)
    {
      pinocchio::DeviceSync device (robot_);
      device.currentConfiguration (config);
      device.computeForwardKinematics ();
      device.updateGeometryPlacements ();

      fcl::CollisionResult collisionResult;
      CollisionPairs_t::const_iterator _col;
      if (collide (collisionPairs_, collisionRequest_, collisionResult, _col, device.d())
          ||
          ( checkParameterized_ &&
            collide (parameterizedPairs_, collisionRequest_, collisionResult, _col, device.d())
          )) {
        CollisionValidationReportPtr_t report (new CollisionValidationReport);
        report->object1 = _col->first;
        report->object2 = _col->second;
        report->result = collisionResult;
        if(computeAllContacts_){
          // create report with the first collision
          AllCollisionsValidationReportPtr_t allReport(new AllCollisionsValidationReport);
          allReport->object1 = _col->first;
          allReport->object2 = _col->second;
          allReport->result = collisionResult;
          allReport->collisionReports.push_back(report);
          // then loop over all the remaining pairs :
          ++_col;
          while(_col != collisionPairs_.end()){
            if (collide (_col, collisionRequest_, collisionResult,device.d()) != 0){
              CollisionValidationReportPtr_t report (new CollisionValidationReport);
              report->object1 = _col->first;
              report->object2 = _col->second;
              report->result = collisionResult;
              allReport->collisionReports.push_back(report);
            }
            ++_col;
          }
          validationReport=allReport;
        }else{
          validationReport = report;
        }
        return false;
      }
      return true;
    }

    void CollisionValidation::addObstacle (const CollisionObjectConstPtr_t& object)
    {
      for (size_type j = 0; j < robot_->nbJoints(); ++j) {
        JointPtr_t joint = robot_->jointAt (j);
        addObstacleToJoint (object, joint, false);
      }
    }


    void CollisionValidation::addObstacleToJoint (const CollisionObjectConstPtr_t& object,
                                     const JointPtr_t& joint, const bool includeChildren)
    {
        BodyPtr_t body = joint->linkedBody ();
        if (body) {
            for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
              // TODO: check the objects are not in same joint
              collisionPairs_.push_back (CollisionPair_t (body->innerObjectAt(o), object));
            }
        }
        if(includeChildren) {
            for(std::size_t i=0; i<joint->numberChildJoints(); ++i){
                addObstacleToJoint (object, joint->childJoint(i),includeChildren);
            }
        }
    }

    struct CollisionPairComparision {
      CollisionPair_t a;
      CollisionPairComparision (const CollisionPair_t& p) : a (p) {}
      bool operator() (const CollisionPair_t& b)
      {
        return (&(a.first ->pinocchio()) == &(b.first ->pinocchio()))
          &&   (&(a.second->pinocchio()) == &(b.second->pinocchio()));
      }
    };

    void CollisionValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle)
    {
      BodyPtr_t body = joint->linkedBody ();
      if (body) {
        for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
          CollisionPair_t colPair (body->innerObjectAt(o), obstacle);
          CollisionPairComparision compare (colPair);
          std::size_t nbDelPairs = 0;
          CollisionPairs_t::iterator _collisionPair (collisionPairs_.begin());
          while ( (_collisionPair = std::find_if (_collisionPair, collisionPairs_.end(), compare))
              != collisionPairs_.end()) {
            _collisionPair = collisionPairs_.erase (_collisionPair);
            ++nbDelPairs;
          }
          if (nbDelPairs == 0) {
            std::ostringstream oss;
            oss << "CollisionValidation::removeObstacleFromJoint: obstacle \""
                << obstacle->name () <<
                "\" is not registered as obstacle for joint \"" << joint->name ()
                << "\".";
            throw std::runtime_error (oss.str ());
          } else if (nbDelPairs >= 2) {
            hppDout (error, "obstacle "<< obstacle->name () <<
                     " was registered " << nbDelPairs
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
      se3::JointIndex j1, j2;
      fcl::CollisionResult unused;
      while (_colPair != collisionPairs_.end ()) {
        j1 = _colPair->first ->jointIndex();
        j2 = _colPair->second->jointIndex();

        switch (matrix(j1, j2)) {
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
              if (fcl::collide (_colPair->first ->fcl (), _colPair->second->fcl (),
                    collisionRequest_, unused) != 0) {
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

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      collisionRequest_(),
      robot_ (robot),
      parameterizedPairs_(), disabledPairs_(),
      checkParameterized_(false),
      computeAllContacts_(false)
    {
      const se3::GeometryModel& model = robot->geomModel();
      const se3::GeometryData & data  = robot->geomData();

      for (std::size_t i = 0; i < model.collisionPairs.size(); ++i)
        if (data.activeCollisionPairs[i])
          collisionPairs_.push_back(
              makeCollisionPair(robot, model.collisionPairs[i])
              );
    }

  } // namespace core
} // namespace hpp
