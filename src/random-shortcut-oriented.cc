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
#include <hpp/core/random-shortcut-oriented.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/kinodynamic-oriented-path.hh>

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

    RandomShortcutOrientedPtr_t
    RandomShortcutOriented::create (const Problem& problem)
    {
      RandomShortcutOriented* ptr = new RandomShortcutOriented (problem);
      return RandomShortcutOrientedPtr_t (ptr);
    }

    RandomShortcutOriented::RandomShortcutOriented (const Problem& problem) :
      RandomShortcut (problem)
    {
    }

    PathVectorPtr_t RandomShortcutOriented::optimize (const PathVectorPtr_t& path)
    {
      using std::numeric_limits;
      using std::make_pair;
      bool finished = false;
      value_type t0, t3;
      const Configuration_t q0 = path->initial ();
      const Configuration_t q3 = path->end ();
      PathVectorPtr_t tmpPath = path;

      // Maximal number of iterations without improvements
      const std::size_t n = problem().getParameter<std::size_t>("PathOptimizersNumberOfLoops", 5);
      std::size_t projectionError = n;
      std::deque <value_type> length (n-1,
                                      numeric_limits <value_type>::infinity ());
      length.push_back (PathLength<>::run (tmpPath, problem ().distance ()));
      PathVectorPtr_t result;
      Configuration_t q1 (path->outputSize ()),
          q2 (path->outputSize ());

      while (!finished && projectionError != 0) {
        t0 = tmpPath->timeRange ().first;
        t3 = tmpPath->timeRange ().second;
        value_type u2 = t0 + (t3 -t0) * rand ()/RAND_MAX;
        value_type u1 = t0 + (t3 -t0) * rand ()/RAND_MAX;
        value_type t1, t2;
        if (u1 < u2) {t1 = u1; t2 = u2;} else {t1 = u2; t2 = u1;}
        if (!(*tmpPath) (q1, t1)) {
          hppDout (error, "Configuration at param " << t1 << " could not be "
                   "projected");
          projectionError--;
          continue;
        }
        if (!(*tmpPath) (q2, t2)) {
          hppDout (error, "Configuration at param " << t1 << " could not be "
                   "projected");
          projectionError--;
          continue;
        }
        // Validate sub parts
        bool valid [3];
        PathPtr_t straight [3];
        PathPtr_t oriented;
        KinodynamicPathPtr_t castedPath;
        bool orientedValid(false);
        straight [0] = steer (q0, q1);
        straight [1] = steer (q1, q2);
        straight [2] = steer (q2, q3);
        PathPtr_t proj [3];
        for (unsigned i=0; i<3; ++i) {
          PathPtr_t validPart;
          PathValidationReportPtr_t report;
          if (!straight [i]) valid[i] = false;
          else {
            if (problem().pathProjector()) {
              valid[i] = problem().pathProjector()->apply(straight[i], proj[i]);
              if (!valid[i]) continue;
            } else proj[i] = straight[i];
            valid [i] = problem ().pathValidation ()->validate(proj [i], false, validPart, report);
            if(valid[i]){
              castedPath = boost::dynamic_pointer_cast<KinodynamicPath>(straight[i]);
              if(castedPath){
                oriented = KinodynamicOrientedPath::createCopy(castedPath);
                orientedValid = problem ().pathValidation ()->validate(oriented, false, validPart, report);
                if(orientedValid)
                  proj[i] = oriented;
              }
            }
          }
        }
        // Replace valid parts
        result = PathVector::create (path->outputSize (),
                                     path->outputDerivativeSize ());
        try {
          if (valid [0])
            result->appendPath (proj [0]);
          else
            result->concatenate (*(tmpPath->extract
                                   (make_pair <value_type,value_type> (t0, t1))->
                                   as <PathVector> ()));
          if (valid [1])
            result->appendPath (proj [1]);
          else
            result->concatenate (*(tmpPath->extract
                                   (make_pair <value_type,value_type> (t1, t2))->
                                   as <PathVector> ()));
          if (valid [2])
            result->appendPath (proj [2]);
          else
            result->concatenate (*(tmpPath->extract
                                   (make_pair <value_type, value_type> (t2, t3))->
                                   as <PathVector> ()));
        } catch (const projection_error& e) {
          hppDout (error, "Caught exception at with time " << t1 << " and " <<
                   t2 << ": " << e.what ());
          projectionError--;
          result = tmpPath;
          continue;
        }
        length.push_back (PathLength<>::run (result, problem ().distance ()));
        length.pop_front ();
        finished = (length [0] - length [n-1]) <= 1e-4 * length[n-1];
        hppDout (info, "length = " << length [n-1]);
        tmpPath = result;
      }
      hppDout (info, "RandomShortcutOriented:" << *result);
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

