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

# include <hpp/model/collision-object.hh>
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
      /// First object in collision
      CollisionObjectPtr_t object1;
      /// Second object in collision
      CollisionObjectPtr_t object2;
      /// fcl collision results
      fcl::CollisionResult result;
      /// Write report in a stream
      virtual std::ostream& print (std::ostream& os) const
      {
	os << "Collision between object " << object1->name () << " and "
	   << object2->name ();
	return os;
      }
    }; // class CollisionValidationReport

    /// Validate a configuration with respect to collision
    ///
    struct HPP_CORE_DLLAPI AllCollisionsValidationReport : public CollisionValidationReport
    {
      std::vector<CollisionValidationReportPtr_t> collisionReports;
      virtual std::ostream& print (std::ostream& os) const
      {
        os <<" Number of collisions : "<<collisionReports.size()<<".";
        for(std::vector<CollisionValidationReportPtr_t>::const_iterator it = collisionReports.begin() ; it != collisionReports.end() ; ++it){
          (*it)->print(os);
        }
        return os;
      }
    }; // class AllCollisionValidationReport
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COLLISION_VALIDATION_REPORT_HH
