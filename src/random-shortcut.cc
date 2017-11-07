//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#include <limits>
#include <deque>
#include <cstdlib>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/path-projector.hh>

namespace hpp {
  namespace core {
    // Compute the length of a vector of paths assuming that each element
    // is optimal for the given distance.
    template <bool reEstimateLength = false> struct PathLength {
      static inline value_type run (const PathVectorPtr_t& path,
                             const DistancePtr_t& distance)
      {
        if (reEstimateLength) return path->length ();
        else {
          value_type result = 0;
          for (std::size_t i=0; i<path->numberPaths (); ++i) {
            const PathPtr_t& element (path->pathAtRank (i));
            Configuration_t q1 = element->initial ();
            Configuration_t q2 = element->end ();
            result += (*distance) (q1, q2);
          }
          return result;
        }
      }
    };

    RandomShortcutPtr_t
    RandomShortcut::create (const Problem& problem)
    {
      RandomShortcut* ptr = new RandomShortcut (problem);
      return RandomShortcutPtr_t (ptr);
    }

    RandomShortcut::RandomShortcut (const Problem& problem) :
      PathOptimizer (problem)
    {
    }

    PathVectorPtr_t RandomShortcut::optimize (const PathVectorPtr_t& path)
    {
      using std::numeric_limits;
      using std::make_pair;
      bool finished = false;
      value_type t[4];
      Configuration_t q[4];
      q[0] = path->initial();
      q[1].resize(path->outputSize ()),
      q[2].resize(path->outputSize ());
      q[3] = path->end();
      PathVectorPtr_t tmpPath = path;

      // Maximal number of iterations without improvements
      const std::size_t n = problem().getParameter<std::size_t>("PathOptimizersNumberOfLoops", 5);
      std::size_t projectionError = n;
      std::deque <value_type> length (n-1,
				      numeric_limits <value_type>::infinity ());
      length.push_back (PathLength<>::run (tmpPath, problem ().distance ()));
      PathVectorPtr_t result;

      while (!finished && projectionError != 0) {
        t[0] = tmpPath->timeRange ().first;
        t[3] = tmpPath->timeRange ().second;
	value_type u2 = (t[3]-t[0]) * rand ()/RAND_MAX;
	value_type u1 = (t[3]-t[0]) * rand ()/RAND_MAX;
	if (u1 < u2) {
          t[1] = t[0] + u1; t[2] = t[0] + u2;
        } else {
          t[1] = t[0] + u2; t[2] = t[0] + u1;
        }
        bool error = false;
        for (int i = 1; i < 3; ++i) {
          if (!(*tmpPath) (q[i], t[i])) {
            hppDout (error, "Configuration at param "
                << t[i] << " could not be projected");
            projectionError--;
            error = true;
            break;
          }
        }
        if (error) continue;
	// Validate sub parts
	bool valid [3];
	PathPtr_t proj [3];
        // Build and projects the path
        for (int i = 0; i < 3; ++i)
          proj[i] = steer (q[i], q[i+1]);
        if (!proj[0] && !proj[1] && !proj[2]) {
          hppDout (info, "Enable to create a valid path");
          projectionError--;
          continue;
        }
        // validate the paths
        for (unsigned i=0; i<3; ++i) {
	  PathPtr_t validPart;
	  PathValidationReportPtr_t report;
          if (!proj [i]) valid[i] = false;
          else
            valid [i] = problem ().pathValidation ()->validate
              (proj [i], false, validPart, report);
	}
	// Replace valid parts
	result = PathVector::create (path->outputSize (),
				     path->outputDerivativeSize ());
        try {
          for (int i = 0; i < 3; ++i) {
            if (valid [i])
              result->appendPath (proj [i]);
            else
              result->concatenate (tmpPath->extract
                                   (make_pair (t[i], t[i+1]))->
                                   as <PathVector> ());
          }
        } catch (const projection_error& e) {
          hppDout (error, "Caught exception at with time " << t[1] << " and " <<
              t[2] << ": " << e.what ());
          projectionError--;
          result = tmpPath;
          continue;
        }
        value_type newLength = PathLength<>::run (result, problem ().distance ());
        if (length[n-1] <= newLength) {
          hppDout (info,  "the length would increase:" << length[n-1] << " " << newLength);
          result = tmpPath;
          projectionError--;
        } else {
          length.push_back (newLength);
          length.pop_front ();
          finished = (length [0] - length [n-1]) <= 1e-4 * length[n-1];
          hppDout (info, "length = " << length [n-1]);
          tmpPath = result;
          projectionError = n;
        }
      }
      if (!result) return path;
      hppDout (info, "RandomShortcut:" << *result);
      for (std::size_t i = 0; i < result->numberPaths (); ++i) {
        if (result->pathAtRank(i)->constraints())
          hppDout (info, "At rank " << i << ", constraints are " <<
              *result->pathAtRank(i)->constraints());
        else
          hppDout (info, "At rank " << i << ", no constraints");
      }
      return result;
    }
  } // namespace core
} // namespace hpp
