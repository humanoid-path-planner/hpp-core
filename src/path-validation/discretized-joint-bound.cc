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

#include <hpp/core/path-validation/discretized.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/joint-bound-validation.hh>

namespace hpp {
  namespace core {
    namespace pathValidation {
      DiscretizedPtr_t createDiscretizedJointBound (
          const DevicePtr_t& robot, const value_type& stepSize)
      {
        DiscretizedPtr_t pv (Discretized::create (stepSize));
        JointBoundValidationPtr_t jbv (JointBoundValidation::create (robot));
        pv->add (jbv);
        return pv;
      }
    } // namespace pathValidation
  } // namespace core
} // namespace hpp
