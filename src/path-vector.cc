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

#include <hpp/core/path-vector.hh>

#include <stdexcept>

namespace hpp {
  namespace core {

    std::size_t PathVector::rankAtParam (const value_type& param,
					 value_type& localParam) const
    {
      assert(!timeParameterization());
      if (paths_.empty())
        throw std::runtime_error ("PathVector is empty.");
      std::size_t res = 0;
      localParam = param;
      bool finished = false;

      while (res + 1 < paths_.size () && !finished) {
	if (localParam > paths_ [res]->length ()) {
	  localParam -= paths_ [res]->length ();
	  res ++;
	}
	else {
	  finished = true;
	}
      }
      if (localParam > paths_ [res]->length ()) {
	if (res != paths_.size () -1) {
	  throw std::runtime_error ("localparam out of range.");
	}
	localParam = paths_ [res]->timeRange ().second;
      } else {
	localParam += paths_ [res]->timeRange ().first;
      }
      assert (localParam >= paths_ [res]->timeRange ().first -
	      std::numeric_limits <float>::epsilon ());
      assert (localParam <= paths_ [res]->timeRange ().second +
	      std::numeric_limits <float>::epsilon ());
      return res;
    }

    void PathVector::appendPath (const PathPtr_t& path)
    {
      assert(!timeParameterization());
      paths_.push_back (path);
      interval_t tr = timeRange();
      tr.second += path->length ();
      timeRange (tr);
    }

    PathPtr_t PathVector::pathAtRank (std::size_t rank) const
    {
      PathPtr_t copy;
      if (constraints ()) {
	if (paths_ [rank]->constraints ()) {
	  throw std::runtime_error
	    ("Attempt to extract a path from a path vector where both "
	     "are subject to constraints. This is not supported.");
	} else {
	  ConstraintPtr_t constraintCopy (constraints ()->copy ());
	  HPP_STATIC_CAST_REF_CHECK (ConstraintSet, *constraintCopy);
	  copy = paths_ [rank]->copy (HPP_STATIC_PTR_CAST (ConstraintSet,
							   constraintCopy));
	}
      } else {
	copy = paths_ [rank]->copy ();
      }
      return copy;
    }

    void PathVector::concatenate (const PathVector& path)
    {
      for (std::size_t i=0; i<path.numberPaths (); ++i) {
	appendPath (path.pathAtRank (i)->copy ());
      }
    }

    void PathVector::concatenate (const PathVectorPtr_t& path)
    {
      for (std::size_t i=0; i<path->numberPaths (); ++i) {
	appendPath (path->pathAtRank (i)->copy ());
      }
    }

    void PathVector::flatten (PathVectorPtr_t p) const
    {
      assert(!timeParameterization());
      for (std::size_t i = 0; i < numberPaths (); ++i) {
        PathPtr_t path = pathAtRank (i);
        PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST(PathVector, path);
        if (pv) pv->flatten (p);
        else    p->appendPath (path);
      }
    }

    bool PathVector::impl_compute (ConfigurationOut_t result,
				   value_type param) const
    {
      assert(!timeParameterization());
      // Find direct path in vector corresponding to parameter.
      size_t rank;
      value_type localParam;
      rank = rankAtParam (param, localParam);

      PathPtr_t subpath = paths_ [rank];
      return (*subpath) (result, localParam);
    }

    void PathVector::impl_derivative (vectorOut_t result, const value_type& param,
				      size_type order) const
    {
      assert(!timeParameterization());
      // Find direct path in vector corresponding to parameter.
      size_t rank;
      value_type localParam;
      rank = rankAtParam (param, localParam);

      PathPtr_t subpath = paths_ [rank];
      subpath->derivative (result, localParam, order);
    }

    PathPtr_t PathVector::impl_extract (const interval_t& subInterval) const
    {
      assert(!timeParameterization());
      using std::make_pair;
      PathVectorPtr_t path = create (outputSize (), outputDerivativeSize ());
      bool reversed = subInterval.first > subInterval.second ? true : false;
      if (reversed) {
	value_type tmin = subInterval.second;
	value_type tmax = subInterval.first;
	value_type localtmin, localtmax;
	std::size_t imin = rankAtParam (tmin, localtmin);
	std::size_t imax = rankAtParam (tmax, localtmax);
	value_type t1min, t1max;
	long int i = imax; // i should be a signed int otherwise the loop below
	                   // fails.
	do {
	  // t1max >= t1min
	  t1min = paths_ [i]->timeRange ().second;
	  t1max = paths_ [i]->timeRange ().first;
	  if (i == (long int) imax) {
	    t1min = localtmax;
	  }
	  if (i == (long int) imin) {
	    t1max = localtmin;
	  }
	  assert (t1max - paths_ [i]->timeRange ().first >
		  -std::numeric_limits <float>::epsilon ());
	  assert (t1min - paths_ [i]->timeRange ().second <
		  std::numeric_limits <float>::epsilon ());
	  t1min = std::max (t1min, paths_ [i]->timeRange ().first);
	  t1max = std::min (t1max, paths_ [i]->timeRange ().second);
          if (t1min != t1max) {
            path->appendPath (paths_ [i]->extract (make_pair (t1min, t1max)));
          }
	  --i;
	} while (i >= (long int) imin);
      } else {
	value_type tmin = subInterval.first;
	value_type tmax = subInterval.second;
	value_type localtmin, localtmax;
	std::size_t imin = rankAtParam (tmin, localtmin);
	std::size_t imax = rankAtParam (tmax, localtmax);
	value_type t1min, t1max;
	std::size_t i = imin;
	do {
	  // t1max >= t1min
	  t1min = paths_ [i]->timeRange ().first;
	  t1max = paths_ [i]->timeRange ().second;
	  if (i == imin) {
	    t1min = localtmin;
	  }
	  if (i == imax) {
	    t1max = localtmax;
	  }
	  // Check numerical precision
	  assert (t1min - paths_ [i]->timeRange ().first >
		  -std::numeric_limits <float>::epsilon ());
	  assert (t1max - paths_ [i]->timeRange ().second <
		  std::numeric_limits <float>::epsilon ());
	  t1min = std::max (t1min, paths_ [i]->timeRange ().first);
	  t1max = std::min (t1max, paths_ [i]->timeRange ().second);
          if (t1min != t1max) {
            path->appendPath (paths_ [i]->extract (make_pair (t1min, t1max)));
          }
	  ++i;
	} while (i <= imax);
      }
      return path;
    }
  } //   namespace core
} // namespace hpp
