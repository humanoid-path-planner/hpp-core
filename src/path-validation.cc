//
// Copyright (c) 2022 CNRS Airbus S.A.S.
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

#include <hpp/core/path-validation.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/pinocchio/liegroup-space.hh>

namespace hpp {
  namespace core {
    bool PathValidation::validate(ConfigurationIn_t q,
                                  ValidationReportPtr_t& report)
    {
      // Create a straight path of length 0 with the configuration.
      // The output space does not matter here since no Liegroup operation
      // will be performed. Thus, we use Rn.
      StraightPathPtr_t p(StraightPath::create
        (LiegroupSpace::Rn(q.size()),q, q, std::make_pair<value_type,
         value_type>(0,0)));
      PathPtr_t unused;
      PathValidationReportPtr_t r;
      bool res(this->validate(p, false, unused, r));
      if (r) report = r->configurationReport;
      return res;
    }
  } // namespace core
} // namespace hpp
