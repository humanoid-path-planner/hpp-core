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

namespace hpp {
  namespace core {
    // Compute the length of a vector of paths assuming that each element
    // is optimal for the given distance.
    static value_type pathLength (const PathVectorPtr_t& path,
				  const DistancePtr_t& distance)
    {
      value_type result = 0;
      for (std::size_t i=0; i<path->numberPaths (); ++i) {
	const PathPtr_t& element (path->pathAtRank (i));
	Configuration_t q1 = element->initial ();
	Configuration_t q2 = element->end ();
	result += (*distance) (q1, q2);
      }
      return result;
    }

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
      value_type t3 = path->timeRange ().second;
      Configuration_t q0 = path->initial ();
      Configuration_t q3 = path->end ();
      PathVectorPtr_t tmpPath = path;

      // Maximal number of iterations without improvements
      const std::size_t n = 5;
      std::deque <value_type> length (n-1,
				      numeric_limits <value_type>::infinity ());
      length.push_back (pathLength (tmpPath, problem ().distance ()));
      PathVectorPtr_t result;
      Configuration_t q1 (path->outputSize ()),
                      q2 (path->outputSize ());

      while (!finished) {
	t3 = tmpPath->timeRange ().second;
	value_type u2 = t3 * rand ()/RAND_MAX;
	value_type u1 = t3 * rand ()/RAND_MAX;
	value_type t1, t2;
	if (u1 < u2) {t1 = u1; t2 = u2;} else {t1 = u2; t2 = u1;}
	if (!(*tmpPath) (q1, t1)) {
          hppDout (error, "Configuration at param " << t1 << " could not be "
              "projected");
        }
	if (!(*tmpPath) (q2, t2)) {
          hppDout (error, "Configuration at param " << t1 << " could not be "
              "projected");
        }
	// Validate sub parts
	bool valid [3];
	PathPtr_t straight [3];
	straight [0] = steer (q0, q1);
	straight [1] = steer (q1, q2);
	straight [2] = steer (q2, q3);
	for (unsigned i=0; i<3; ++i) {
	  PathPtr_t validPart;
	  PathValidationReportPtr_t report;
          if (!straight [i]) valid[i] = false;
          else {
            valid [i] = problem ().pathValidation ()->validate
              (straight [i], false, validPart, report);
          }
	}
	// Replace valid parts
	result = PathVector::create (path->outputSize (),
				     path->outputDerivativeSize ());
	if (valid [0])
	  result->appendPath (straight [0]);
	else
	  result->concatenate (*(tmpPath->extract
				 (make_pair <value_type,value_type> (0, t1))->
				 as <PathVector> ()));
	if (valid [1])
	  result->appendPath (straight [1]);
	else
	  result->concatenate (*(tmpPath->extract
				 (make_pair <value_type,value_type> (t1, t2))->
				 as <PathVector> ()));
	if (valid [2])
	  result->appendPath (straight [2]);
	else
	  result->concatenate (*(tmpPath->extract
				(make_pair <value_type, value_type> (t2, t3))->
				 as <PathVector> ()));
	length.push_back (pathLength (result, problem ().distance ()));
	length.pop_front ();
	finished = (length [0] <= length [n-1]);
	hppDout (info, "length = " << length [n-1]);
	tmpPath = result;
      }
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
