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
      std::size_t iPair = 0;
      const ObstacleUser::CollisionPairs_t* pairs (&cPairs_);
      bool collide = ObstacleUser::collide (cPairs_, cRequests_,
          collisionResult, iPair, device.d());
      if (!collide && checkParameterized_) {
        collide = ObstacleUser::collide (pPairs_, pRequests_,
            collisionResult, iPair, device.d());
        pairs = &pPairs_;
      }
      if (collide) {
        CollisionValidationReportPtr_t report (
            new CollisionValidationReport ((*pairs)[iPair], collisionResult));

        if(computeAllContacts_){
          // create report with the first collision
          AllCollisionsValidationReportPtr_t allReport(
              new AllCollisionsValidationReport ((*pairs)[iPair], collisionResult));
          allReport->collisionReports.push_back(report);
          // then loop over all the remaining pairs :
          ++iPair;
          for (;iPair < cPairs_.size(); ++iPair) {
            collisionResult.clear();
            if (fcl::collide (
                  cPairs_[iPair].first ->fcl (device.d()),
                  cPairs_[iPair].second->fcl (device.d()),
                  cRequests_[iPair], collisionResult) != 0) {
              CollisionValidationReportPtr_t report (
                  new CollisionValidationReport ((*pairs)[iPair], collisionResult));
              allReport->collisionReports.push_back(report);
            }
          }
          validationReport=allReport;
        }else{
          validationReport = report;
        }
        return false;
      }
      return true;
    }

    CollisionValidation::CollisionValidation (const DevicePtr_t& robot) :
      ObstacleUser (robot),
      robot_ (robot),
      checkParameterized_(false),
      computeAllContacts_(false)
    {
      fcl::CollisionRequest req (fcl::NO_REQUEST,1); 
      req.enable_cached_gjk_guess = true;
      defaultRequest() = req;
      addRobotCollisionPairs();
    }

  } // namespace core
} // namespace hpp
