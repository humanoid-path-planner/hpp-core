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

#include <boost/serialization/vector.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <hpp/util/indent.hh>
#include <hpp/util/serialization.hh>

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

    inline const value_type& Iinit(const interval_t& I, bool reverse) { return (reverse ? I.second : I.first); }
    inline const value_type& Iend (const interval_t& I, bool reverse) { return (reverse ? I.first : I.second); }

    PathPtr_t PathVector::impl_extract (const interval_t& subInterval) const
    {
      assert(!timeParameterization());
      PathVectorPtr_t path = create (outputSize (), outputDerivativeSize ());
      bool reversed = subInterval.first > subInterval.second ? true : false;

      value_type localtinit, localtend;
      std::size_t iinit = rankAtParam (subInterval.first , localtinit),
                  iend  = rankAtParam (subInterval.second, localtend );
      if (iinit == iend) {
        path->appendPath(paths_[iinit]->extract(
              localtinit,
              localtend));
      } else {
        path->appendPath(paths_[iinit]->extract(
              localtinit,
              Iend(paths_[iinit]->timeRange(), reversed)));
        int one = (reversed ? -1 : 1);
        for (std::size_t i = iinit + one;
            (reversed && i > iend) || i < iend;
            i += one)
          path->appendPath(reversed ?  paths_[i]->reverse() : paths_[i]);
        path->appendPath(paths_[iend]->extract(
              Iinit(paths_[iend]->timeRange(), reversed),
              localtend));
      }
      return path;
    }

    PathPtr_t PathVector::reverse () const
    {
      assert(!timeParameterization());
      PathVectorPtr_t ret = create (outputSize (), outputDerivativeSize ());
      std::for_each (paths_.rbegin(), paths_.rend(), [&ret](const PathPtr_t& path) {
        ret->appendPath(path->reverse());
      });
      return ret;
    }

    std::ostream& PathVector::print (std::ostream &os) const
    {
      Path::print (os << "PathVector:") << incendl;
      for (Paths_t::const_iterator itPath = paths_.begin ();
          itPath != paths_.end (); ++itPath) {
        os << (**itPath) << iendl;
      }
      return os << decindent;
    }

    template<class Archive>
    void PathVector::serialize(Archive & ar, const unsigned int version)
    {
      using namespace boost::serialization;
      (void) version;
      ar & make_nvp("base", base_object<Path>(*this));
      ar & BOOST_SERIALIZATION_NVP(paths_);
      ar & BOOST_SERIALIZATION_NVP(weak_);
    }

    HPP_SERIALIZATION_IMPLEMENT(PathVector);
  } //   namespace core
} // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::core::PathVector)
