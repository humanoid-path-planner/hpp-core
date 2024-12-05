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

#ifndef HPP_CORE_COLLISION_VALIDATION_REPORT_HH
#define HPP_CORE_COLLISION_VALIDATION_REPORT_HH

#include <coal/collision_data.h>

#include <hpp/core/collision-pair.hh>
#include <hpp/core/validation-report.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/util/indent.hh>

namespace hpp {
namespace core {
/// \addtogroup validation
/// \{

/// Validate a configuration with respect to collision
///
struct HPP_CORE_DLLAPI CollisionValidationReport : public ValidationReport {
  CollisionValidationReport() {}

  CollisionValidationReport(CollisionObjectConstPtr_t o1,
                            CollisionObjectConstPtr_t o2,
                            const coal::CollisionResult& r)
      : object1(o1), object2(o2), result(r) {}

  CollisionValidationReport(const CollisionPair_t& pair,
                            const coal::CollisionResult& r)
      : object1(pair.first), object2(pair.second), result(r) {}

  /// First object in collision
  CollisionObjectConstPtr_t object1;
  std::string objectName1;
  /// Second object in collision
  CollisionObjectConstPtr_t object2;
  std::string objectName2;
  /// coal collision results
  coal::CollisionResult result;
  /// Write report in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "Collision between object "
       << (object1 ? object1->name() : objectName1) << " and "
       << (object2 ? object2->name() : objectName2);
    return os;
  }
  std::pair<std::string, std::string> getObjectNames() const {
    return std::pair<std::string, std::string>(
        object1 ? object1->name() : objectName1,
        object2 ? object2->name() : objectName2);
  }
};  // class CollisionValidationReport

/// Validate a configuration with respect to collision
///
struct HPP_CORE_DLLAPI AllCollisionsValidationReport
    : public CollisionValidationReport {
  AllCollisionsValidationReport() {}

  AllCollisionsValidationReport(CollisionObjectConstPtr_t o1,
                                CollisionObjectConstPtr_t o2,
                                const coal::CollisionResult& r)
      : CollisionValidationReport(o1, o2, r) {}

  AllCollisionsValidationReport(const CollisionPair_t& pair,
                                const coal::CollisionResult& r)
      : CollisionValidationReport(pair, r) {}

  std::vector<CollisionValidationReportPtr_t> collisionReports;
  virtual std::ostream& print(std::ostream& os) const {
    os << " Number of collisions : " << collisionReports.size() << "."
       << incendl;
    for (std::vector<CollisionValidationReportPtr_t>::const_iterator it =
             collisionReports.begin();
         it != collisionReports.end(); ++it) {
      os << **it << iendl;
    }
    return os << decindent;
  }
};  // class AllCollisionsValidationReport
/// \}
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_COLLISION_VALIDATION_REPORT_HH
