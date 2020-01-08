//
// Copyright (c) 2017 CNRS
// Authors: Diane Bury
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

#include <hpp/pinocchio/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validations.hh>

namespace hpp {
  namespace core {

    PathValidationsPtr_t PathValidations::create ()
    {
      PathValidations* ptr = new PathValidations();
      return PathValidationsPtr_t (ptr);
    }

    void PathValidations::addPathValidation
    (const PathValidationPtr_t& pathValidation)
    {
      validations_.push_back (pathValidation);
    }

    bool PathValidations::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
     PathValidationReportPtr_t& validationReport)
    {
      PathPtr_t tempPath = path;
      PathPtr_t tempValidPart;
      PathValidationReportPtr_t tempValidationReport;

      bool result = true;
      value_type lastValidTime = path->timeRange ().second;
      value_type t = lastValidTime;

      for (std::vector <PathValidationPtr_t>::iterator
	     it = validations_.begin (); it != validations_.end (); ++it) {
	    if ((*it)->validate (tempPath, reverse, tempValidPart, tempValidationReport) == false)
        {
          t = tempValidationReport->getParameter();
          if ( t < lastValidTime ) {
              lastValidTime = t;
              tempPath = tempValidPart;
          }
          result = false;
        }
      }
      validPart = tempPath;
      validationReport->setParameter(lastValidTime);
      return result;
    }

    PathValidations::PathValidations ()
    {
    }
  } // namespace core
} // namespace hpp
