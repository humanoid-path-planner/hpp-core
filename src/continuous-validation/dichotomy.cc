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

#include <hpp/core/continuous-validation/dichotomy.hh>

#include <deque>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/continuous-validation/initializer.hh>

#include "continuous-validation/intervals.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {

      DichotomyPtr_t
      Dichotomy::create (const DevicePtr_t& robot, const value_type& tolerance)
      {
        Dichotomy* ptr = new Dichotomy (robot, tolerance);
        DichotomyPtr_t shPtr (ptr);
        ptr->init(shPtr);
        ptr->initializer_->initialize();
        return shPtr;
      }

      Dichotomy::~Dichotomy ()
      {
      }

      bool Dichotomy::validateStraightPath
      (const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
       PathValidationReportPtr_t& report)
      {
        // start by validating end of path
        bool finished = false;
        bool valid = true;
        setPath(path, reverse);
        Intervals validSubset;
        if (reverse) {
          value_type t (path->timeRange ().second);
          validSubset.unionInterval (std::make_pair (t,t));
          Configuration_t q (path->outputSize ());
          while (!finished) {
            bool success = (*path) (q, t);
            PathValidationReportPtr_t pathReport;
            interval_t interval;
            if (!success || !validateConfiguration (q, t, interval,
                      pathReport)) {
              report = pathReport;
              valid = false;
            } else {
              validSubset.unionInterval (interval);
            }
            finished = (!valid || validSubset.contains (path->timeRange ()));
            // Compute next parameter to check as middle of first non tested
            // interval
            std::list <interval_t>::const_reverse_iterator itIntervals
              (validSubset.list ().rbegin ());
            value_type t0 (itIntervals->first); ++ itIntervals;
            value_type t1 (path->timeRange ().first);
            if (itIntervals != validSubset.list ().rend ()) {
              t1 = itIntervals->second;
            }
            t = .5* (t0 + t1);
          }
          if (!valid) {
            assert (validSubset.list ().rbegin ()->first <=
              path->timeRange ().second);
            assert (validSubset.list ().rbegin ()->second >=
              path->timeRange ().second);
            value_type tmin (validSubset.list ().rbegin ()->first);
            value_type tmax (path->timeRange ().second);
            validPart = path->extract (std::make_pair (tmin, tmax));
            return false;
          } else {
            validPart = path;
            return true;
          }
        } else { // reverse
          value_type t (path->timeRange ().first);
          validSubset.unionInterval (std::make_pair (t,t));
          Configuration_t q (path->outputSize ());
          while (!finished) {
            bool success = (*path) (q, t);
            PathValidationReportPtr_t pathReport;
            interval_t interval;
            if (!success || !validateConfiguration (q, t, interval,
                      pathReport)) {
              report = pathReport;
              valid = false;
            } else {
              validSubset.unionInterval (interval);
            }
            finished = (!valid || validSubset.contains (path->timeRange ()));
            // Compute next parameter to check as middle of first non tested
            // interval
            std::list <interval_t>::const_iterator itIntervals
              (validSubset.list ().begin ());
            value_type t0 (itIntervals->second); ++ itIntervals;
            value_type t1 (path->timeRange ().second);
            if (itIntervals != validSubset.list ().end ()) {
              t1 = itIntervals->first;
            }
            t = .5* (t0 + t1);
          }
          if (!valid) {
            assert (validSubset.list ().begin ()->first <=
              path->timeRange ().first);
            assert (validSubset.list ().begin ()->second >=
              path->timeRange ().first);
            value_type tmin (path->timeRange ().first);
            value_type tmax (validSubset.list ().begin ()->second);
            validPart = path->extract (std::make_pair (tmin, tmax));
            return false;
          } else {
            validPart = path;
            return true;
          }
        }
        return false;
      }

      void Dichotomy::init(const DichotomyWkPtr_t weak)
      {
        ContinuousValidation::init (weak);
        weak_ = weak;
      }

      Dichotomy::Dichotomy (const DevicePtr_t& robot, const value_type& tolerance):
	      ContinuousValidation (robot, tolerance), weak_()
      {
        // Tolerance should be equal to 0, otherwise end of valid
        // sub-path might be in collision.
        if (tolerance != 0) {
          throw std::runtime_error ("Dichotomy path validation method does not"
                  "support penetration.");
        }
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
