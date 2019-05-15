//
// Copyright (c) 2019 CNRS
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

#include <hpp/pinocchio/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <hpp/util/debug.hh>

namespace hpp {
  namespace core {
    namespace pathValidation {
      HPP_PREDEF_CLASS (NoValidation);
      typedef boost::shared_ptr <NoValidation> NoValidationPtr_t;
      class NoValidation : public PathValidation
      {
      public:
        // Validate all paths
        virtual bool validate (const PathPtr_t& path, bool,
                               PathPtr_t& validPart,
                               PathValidationReportPtr_t&)
        {
          validPart = path;
          return true;
        }
        static NoValidationPtr_t create (const DevicePtr_t&,
                                         const value_type&)
        {
          NoValidation* ptr = new NoValidation ();
          NoValidationPtr_t shPtr (ptr);
          return shPtr;
        }
      protected:
        NoValidation () {}
      }; // class NoValidation
    } // namespace pathValidation
  } // namespace core
} // namespace hpp
