//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
      const CollisionPairs_t* pairs (&cPairs_);
      CollisionRequests_t* requests (&cRequests_);

      bool collide = ObstacleUser::collide (cPairs_, cRequests_,
          collisionResult, iPair, device.d());
      if (!collide && checkParameterized_) {
        collide = ObstacleUser::collide (pPairs_, pRequests_,
            collisionResult, iPair, device.d());
        pairs = &pPairs_;
        requests = &pRequests_;
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
          for (;iPair < pairs->size(); ++iPair) {
            collisionResult.clear();
            if ((*pairs)[iPair].collide(device.d(),
                  (*requests)[iPair], collisionResult) != 0) {
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
