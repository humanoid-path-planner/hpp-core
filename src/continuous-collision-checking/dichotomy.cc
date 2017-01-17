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

#include <hpp/core/continuous-collision-checking/dichotomy.hh>

#include <deque>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-collision-checking/body-pair-collision.hh"

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {

      DichotomyPtr_t
      Dichotomy::create (const DevicePtr_t& robot, const value_type& tolerance)
      {
	Dichotomy* ptr =
	  new Dichotomy (robot, tolerance);
	DichotomyPtr_t shPtr (ptr);
	return shPtr;
      }

      Dichotomy::~Dichotomy ()
      {
      }

      bool Dichotomy::validateStraightPath
      (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
       PathValidationReportPtr_t& report)
      {
	return false;
      }

      Dichotomy::Dichotomy
      (const DevicePtr_t& robot, const value_type& tolerance) :
	ContinuousCollisionChecking (robot, tolerance)
      {
        // Tolerance should be equal to 0, otherwise end of valid
        // sub-path might be in collision.
        if (tolerance != 0) {
          throw std::runtime_error ("Dichotomy path validation method does not"
				    "support penetration.");
        }
      }
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
