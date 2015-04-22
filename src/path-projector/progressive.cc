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

#include "hpp/core/path-projector/progressive.hh"

#include <hpp/core/path-vector.hh>
#include <hpp/core/config-projector.hh>

#include <limits>
#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      Progressive::Progressive (const core::DistancePtr_t d, value_type step) :
        PathProjector (d), step_ (step)
      {}

      bool Progressive::impl_apply (const PathPtr_t& path,
				    PathPtr_t& proj) const
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

      bool Progressive::applyToStraightPath (const StraightPathPtr_t& path,
					     PathPtr_t& projection) const
      {
        ConstraintSetPtr_t constraints = path->constraints ();
	if (!constraints) {
	  projection = path;
	  return true;
	}
        const ConfigProjectorPtr_t& cp = constraints->configProjector ();
        core::interval_t timeRange = path->timeRange ();
        const Configuration_t& q1 = path->initial ();
        const Configuration_t& q2 = path->end ();
        const size_t maxDichotomyTries = 10,
                     maxPathSplit =
	  (size_t)(10 * (timeRange.second - timeRange.first) / (double)step_);
	assert (constraints->isSatisfied (q1));
        if (!constraints->isSatisfied (q2)) return false;
        if (!cp) {
          projection = path;
          return true;
        }

        bool pathIsFullyProjected = false;
        std::queue <core::StraightPathPtr_t> paths;
        StraightPathPtr_t toSplit =
            core::StraightPath::create (path->device (), q1, q2, d (q1, q2));
        Configuration_t qi (q1.size());
        value_type curStep, curLength;
        size_t c = 0;
        while (true) {
          if (toSplit->length () < step_) {
            paths.push (toSplit);
            pathIsFullyProjected = true;
            break;
          }
          const Configuration_t& qb = toSplit->initial ();
          curStep = step_;
          curLength = std::numeric_limits <value_type>::max();
          size_t dicC = 0;
          /// Find the good length.
          /// Here, it would be good to have an upper bound of the Hessian
          /// of the constraint.
          do {
            if (dicC >= maxDichotomyTries) break;
            (*toSplit) (qi, curStep);
            if (constraints->apply (qi))
              curLength = d (qb, qi);
            curStep /= 2;
            dicC++;
          } while (curLength > step_ || curLength < 1e-3);
          if (dicC >= maxDichotomyTries || c > maxPathSplit) break;
          StraightPathPtr_t part =
            core::StraightPath::create (path->device (), qb, qi, curLength);
          paths.push (part);
          toSplit =
            core::StraightPath::create (path->device (), qi, q2, d (qi, q2));
          c++;
        }
        switch (paths.size ()) {
          case 0:
            timeRange = path->timeRange();
            projection = path->extract
	      (std::make_pair (timeRange.first, timeRange.first));
            return false;
            break;
          case 1:
            projection = paths.front ()->copy (constraints);
            break;
          default:
            core::PathVectorPtr_t pv = core::PathVector::create
	      (path->outputSize (), path->outputDerivativeSize ());
            qi = q1;
            while (!paths.empty ()) {
              assert ((qi - paths.front ()->initial ()).isZero ());
              qi = paths.front ()->end ();
              pv->appendPath (paths.front ()->copy (constraints));
              paths.pop ();
            }
            projection = pv;
            break;
        }
        assert ((projection->initial () - path->initial ()).isZero());
        assert (!pathIsFullyProjected ||
		(projection->end () - path->end ()).isZero());
        return pathIsFullyProjected;
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
