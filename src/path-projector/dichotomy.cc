// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/core/path-projector/dichotomy.hh"

#include <hpp/core/path-vector.hh>
#include <hpp/core/config-projector.hh>

#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      Dichotomy::Dichotomy (const core::DistancePtr_t d, value_type maxPathLength) :
        PathProjector (d), maxPathLength_ (maxPathLength)
      {}

      bool Dichotomy::impl_apply (const PathPtr_t& path, PathPtr_t& proj) const
      {
	assert (path);
	bool success = false;
	PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, path);
	if (!pv) {
	  StraightPathPtr_t sp = HPP_DYNAMIC_PTR_CAST (StraightPath, path);
	  if (!sp) throw std::invalid_argument
		     ("Unknow inherited class of Path");
	  success = applyToStraightPath (sp, proj);
	} else {
	  PathVectorPtr_t res = PathVector::create
	    (pv->outputSize (), pv->outputDerivativeSize ());
	  PathPtr_t part;
	  success = true;
	  for (size_t i = 0; i < pv->numberPaths (); i++) {
	    if (!apply (pv->pathAtRank (i), part)) {
	      // We add the path only if part is not NULL and:
	      // - either its length is not zero,
	      // - or it's not the first one.
	      if (part && (part->length () > 0 || i == 0)) {
		res->appendPath (part);
	      }
	      success = false;
	      break;
	    }
	    res->appendPath (part);
	  }
	  proj = res;
	}
	assert (proj);
	assert ((proj->initial () - path->initial ()).isZero());
	assert (!success || (proj->end () - path->end ()).isZero());
	return success;
      }

      bool Dichotomy::applyToStraightPath (const StraightPathPtr_t& path,
					   PathPtr_t& projection) const
      {
        ConstraintSetPtr_t constraints = path->constraints ();
        const ConfigProjectorPtr_t& cp = constraints->configProjector ();
        core::interval_t timeRange = path->timeRange ();
        const Configuration_t& q1 = (*path)(timeRange.first);
        const Configuration_t& q2 = (*path)(timeRange.second);
        if (cp) cp->rightHandSideFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        if (!cp) {
          projection = path;
          return true;
        }

        bool pathIsFullyProjected = true;
        std::queue <core::StraightPathPtr_t> paths;
        std::stack <core::StraightPathPtr_t> pathToSplit;
        Configuration_t qi (q1.size());
        core::StraightPathPtr_t sPath = core::StraightPath::create
	  (path->device (), q1, q2, d (q1, q2));
        pathToSplit.push (sPath);
        while (!pathToSplit.empty ()) {
          sPath = pathToSplit.top ();
          const StraightPath& sPathRef = *sPath;
          pathToSplit.pop ();
          double l = sPathRef.length ();
          if (l < maxPathLength_) {
            paths.push (sPath);
            continue;
          }
          timeRange = sPathRef.timeRange ();
          const Configuration_t& qb = sPathRef (timeRange.first);
          sPathRef (qi, timeRange.first + l / 2);
          const Configuration_t& qe = sPathRef (timeRange.second);
          if (!constraints->apply (qi)) {
            pathIsFullyProjected = false;
            break;
          }
          StraightPathPtr_t firstPart =
            core::StraightPath::create (path->device (), qb, qi, d (qb, qi));
          StraightPathPtr_t secondPart =
            core::StraightPath::create (path->device (), qi, qe, d (qi, qe));
          if (secondPart->length () == 0 || firstPart->length () == 0) {
            pathIsFullyProjected = false;
            break;
          }
          pathToSplit.push (secondPart);
          pathToSplit.push (firstPart);
        }
        switch (paths.size ()) {
          case 0:
            return false;
            break;
          case 1:
            projection = paths.front ()->copy (constraints);
            break;
          default:
            core::PathVectorPtr_t pv = core::PathVector::create
	      (path->outputSize (), path->outputDerivativeSize ());
            while (!paths.empty ()) {
              pv->appendPath (paths.front ()->copy (constraints));
              paths.pop ();
            }
            projection = pv;
            break;
        }
        return pathIsFullyProjected;
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
