// Copyright (c) 2016, Joseph Mirabel
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

#include "hpp/core/path-projector/recursive-hermite.hh"

#include <hpp/core/path-vector.hh>
#include <hpp/core/hermite-path.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/steering-method/hermite.hh>

#include <limits>
#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      namespace {
      }

      RecursiveHermitePtr_t RecursiveHermite::create (const DistancePtr_t& distance,
          const SteeringMethodPtr_t& steeringMethod, value_type step)
      {
        return RecursiveHermitePtr_t (new RecursiveHermite
            (distance, steeringMethod, step));
      }

      RecursiveHermitePtr_t RecursiveHermite::create (
          const ProblemPtr_t& problem, const value_type& step)
      {
        return create (problem->distance(), problem->steeringMethod(), step);
      }

      RecursiveHermite::RecursiveHermite (const DistancePtr_t& distance,
				const SteeringMethodPtr_t& steeringMethod,
				value_type M) :
        PathProjector (distance, steeringMethod, true), M_ (M)
      {
        assert (HPP_DYNAMIC_PTR_CAST(hpp::core::steeringMethod::Hermite, steeringMethod));
      }

      bool RecursiveHermite::impl_apply (const PathPtr_t& path,
				    PathPtr_t& proj) const
      {
	assert (path);
	bool success = false;
	PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, path);
	if (!pv) {
          if (!path->constraints()
              || !path->constraints()->configProjector()) {
            proj = path;
            success = true;
          } else {
            success = project (path, proj);
          }
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

      bool RecursiveHermite::project (const PathPtr_t& path, PathPtr_t& proj) const
      {
        ConstraintSetPtr_t constraints = path->constraints ();
	if (!constraints) {
	  proj = path;
	  return true;
	}
        const Configuration_t q1 = path->initial ();
        const Configuration_t q2 = path->end ();
        if (!constraints->isSatisfied (q2)) return false;
        const ConfigProjectorPtr_t& cp = constraints->configProjector ();
        if (!cp) {
          proj = path;
          return true;
        }

        const value_type thr = 2 * cp->errorThreshold() / M_;

        HermitePathPtr_t p = HPP_DYNAMIC_PTR_CAST (HermitePath, path);
        if (!p) {
          p = HPP_DYNAMIC_PTR_CAST(HermitePath, steer (path->initial(), path->end()));
        }
        p->computeHermiteLength();
        if (p->hermiteLength() < thr) {
          proj = path;
          return true;
        }
        PathVectorPtr_t res = PathVector::create (path->outputSize(),
                                                  path->outputDerivativeSize());
        if (recurse (p, res, thr)) {
          proj = res;
          return true;
        }
        const value_type tmin = path->timeRange().first;
        switch (res->numberPaths()) {
          case 0:
            proj = path->extract (std::make_pair (tmin, tmin));
            break;
          case 1:
            proj = res->pathAtRank(0);
            break;
          default:
            proj = res;
            break;
        }
        return false;
      }

      bool RecursiveHermite::recurse (const HermitePathPtr_t& path, PathVectorPtr_t& proj, const value_type& thr) const
      {
        if (path->hermiteLength() < thr) {
          proj->appendPath (path);
          return true;
        } else {
          const value_type t = 0.5; //path->timeRange().first + path->length() / 2;
          bool success;
          const Configuration_t q1((*path) (t, success));
          if (!success) {
            return false;
          }
          const Configuration_t q0 = path->initial ();
          const Configuration_t q2 = path->end ();
          // I do not know why we have to divide the velocities by two.
          const vector_t vHalf = path->velocity (t) / 2;

          HermitePathPtr_t left = HPP_DYNAMIC_PTR_CAST(HermitePath, steer (q0, q1));
          left->v0 (path->v0() / 2);
          left->v1 (vHalf);
          left->computeHermiteLength();

          HermitePathPtr_t right = HPP_DYNAMIC_PTR_CAST(HermitePath, steer (q1, q2));
          right->v0 (vHalf);
          right->v1 (path->v1() / 2);
          right->computeHermiteLength();

          const value_type beta = 0.8; // should be between 0.5 and 1.
          // // This is the inverse of the condition in the RSS paper. Is there a typo in the paper ?
          // if (std::max (left->hermiteLength(), right->hermiteLength()) > beta * path->hermiteLength()) return false;
          // if (std::max (left->hermiteLength(), right->hermiteLength()) <= beta * path->hermiteLength()) return false;
          if (!recurse (left, proj, thr)) return false;
          if (!recurse (right, proj, thr)) return false;

          return true;
        }
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
