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

#ifndef HPP_CORE_COLLISION_VALIDATION_REPORT_HH
# define HPP_CORE_COLLISION_VALIDATION_REPORT_HH

# include <hpp/util/indent.hh>

# include <hpp/pinocchio/collision-object.hh>
# include <hpp/core/validation-report.hh>
# include <hpp/fcl/collision_data.h>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// Validate a configuration with respect to collision
    ///
    struct HPP_CORE_DLLAPI CollisionValidationReport : public ValidationReport
    {
      CollisionValidationReport () {}

      CollisionValidationReport (
          CollisionObjectConstPtr_t o1, CollisionObjectConstPtr_t o2,
          const fcl::CollisionResult& r) : object1 (o1), object2 (o2), result(r)
      {}

      CollisionValidationReport (const CollisionPair_t& pair,
          const fcl::CollisionResult& r) :
        object1 (pair.first),
        object2 (pair.second),
        result(r)
      {}

      /// First object in collision
      CollisionObjectConstPtr_t object1;
      std::string objectName1;
      /// Second object in collision
      CollisionObjectConstPtr_t object2;
      std::string objectName2;
      /// fcl collision results
      fcl::CollisionResult result;
      /// Write report in a stream
      virtual std::ostream& print (std::ostream& os) const
      {
	os << "Collision between object " << (object1 ? object1->name() : objectName1) << " and "
	   << (object2 ? object2->name() : objectName2);
	return os;
      }
    }; // class CollisionValidationReport

    /// Validate a configuration with respect to collision
    ///
    struct HPP_CORE_DLLAPI AllCollisionsValidationReport : public CollisionValidationReport
    {
      AllCollisionsValidationReport() {}

      AllCollisionsValidationReport (
          CollisionObjectConstPtr_t o1, CollisionObjectConstPtr_t o2,
          const fcl::CollisionResult& r) : CollisionValidationReport (o1, o2, r)
      {}

      AllCollisionsValidationReport (const CollisionPair_t& pair,
          const fcl::CollisionResult& r) : CollisionValidationReport (pair, r)
      {}

      std::vector<CollisionValidationReportPtr_t> collisionReports;
      virtual std::ostream& print (std::ostream& os) const
      {
        os <<" Number of collisions : "<<collisionReports.size()<<"."<<incendl;
        for(std::vector<CollisionValidationReportPtr_t>::const_iterator it = collisionReports.begin() ; it != collisionReports.end() ; ++it){
          os << **it << iendl;
        }
        return os << decindent;
      }
    }; // class AllCollisionsValidationReport
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COLLISION_VALIDATION_REPORT_HH
