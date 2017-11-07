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

#include <hpp/util/timer.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/steering-method/hermite.hh>

#include <limits>
#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      RecursiveHermitePtr_t RecursiveHermite::create (const DistancePtr_t& distance,
          const SteeringMethodPtr_t& steeringMethod, value_type step)
      {
        value_type beta = 0.9;
        try {
          beta = steeringMethod->problem().getParameter<value_type>
            ("PathProjectionRecursiveHermiteBeta", beta);
          hppDout (info, "beta is " << beta);
        } catch (const boost::bad_any_cast& e) {
          hppDout (error, "Could not cast parameter "
              "PathProjectionRecursiveHermiteBeta to value_type");
        }
        return RecursiveHermitePtr_t (new RecursiveHermite
            (distance, steeringMethod, step, beta));
      }

      RecursiveHermitePtr_t RecursiveHermite::create (
          const Problem& problem, const value_type& step)
      {
        return create (problem.distance(), problem.steeringMethod(), step);
      }

      RecursiveHermite::RecursiveHermite (const DistancePtr_t& distance,
				const SteeringMethodPtr_t& steeringMethod,
				const value_type& M, const value_type& beta) :
        PathProjector (distance, steeringMethod, false), M_ (M),
        beta_ (beta)
      {
        // beta should be between 0.5 and 1.
        if (beta_ < 0.5 || 1 < beta_)
          throw std::invalid_argument ("Beta should be between 0.5 and 1");
        if (!HPP_DYNAMIC_PTR_CAST(hpp::core::steeringMethod::Hermite, steeringMethod))
          throw std::invalid_argument ("Steering method should be of type Hermite");
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
        if (!cp || cp->dimension() == 0) {
          proj = path;
          return true;
        }

        steeringMethod_->constraints(constraints);

        const value_type thr = 2 * cp->errorThreshold() / M_;

        std::vector<HermitePtr_t> ps;
        HermitePtr_t p = HPP_DYNAMIC_PTR_CAST (Hermite, path);
        if (!p) {
          InterpolatedPathPtr_t ip = HPP_DYNAMIC_PTR_CAST(InterpolatedPath, path);
          if (ip) {
            typedef InterpolatedPath::InterpolationPoints_t IPs_t;
            const IPs_t& ips = ip->interpolationPoints();
            ps.reserve(ips.size() - 1);
            IPs_t::const_iterator _ip1 = ips.begin(); std::advance (_ip1, 1);
            for (IPs_t::const_iterator _ip0 = ips.begin();
                _ip1 != ips.end(); ++_ip0) {
              ps.push_back (HPP_DYNAMIC_PTR_CAST(Hermite,
                    steer (_ip0->second, _ip1->second)));
              ++_ip1;
            }
          } else {
            p = HPP_DYNAMIC_PTR_CAST(Hermite, steer (path->initial(), path->end()));
            ps.push_back (p);
          }
        } else {
          ps.push_back (p);
        }
        PathVectorPtr_t res = PathVector::create (path->outputSize(),
                                                  path->outputDerivativeSize());
        bool success = true;
        for (std::size_t i = 0; i < ps.size(); ++i) {
          p = ps[i];
          p->computeHermiteLength();
          if (p->hermiteLength() < thr) {
            res->appendPath (p);
            continue;
          }
          PathVectorPtr_t r = PathVector::create (path->outputSize(),
                                                  path->outputDerivativeSize());
          std::cout << p->hermiteLength() 
            << " / " << thr
            << " : " << 
            path->constraints()->name() << std::endl;
          success = recurse (p, r, thr);
          res->concatenate (r);
          if (!success) break;
        }
#if HPP_ENABLE_BENCHMARK
        value_type min = std::numeric_limits<value_type>::max(), max = 0, totalLength = 0;
        const size_t nbPaths = res->numberPaths();
        for (std::size_t i = 0; i < nbPaths; ++i) {
          PathPtr_t curP = res->pathAtRank(i);
          const value_type l = d(curP->initial(), curP->end());
          if (l < min) min = l;
          else if (l > max) max = l;
          totalLength += l;
        }
        hppBenchmark("Hermite path: "
            << nbPaths
            << ", [ " << min
            <<   ", " << (nbPaths == 0 ? 0 : totalLength / (value_type)nbPaths)
            <<   ", " << max << "]"
            );
#endif
        if (success) {
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

      bool RecursiveHermite::recurse (const HermitePtr_t& path, PathVectorPtr_t& proj,
          const value_type& acceptThr) const
      {
        if (path->hermiteLength() < acceptThr) {
          // TODO this does not work because it is not possible to remove
          // constraints from a path.
          // proj->appendPath (path->copy (ConstraintSetPtr_t()));
          proj->appendPath(path);
          return true;
        } else {
          const value_type t = 0.5; //path->timeRange().first + path->length() / 2;
          bool success;
          const Configuration_t q1((*path) (t, success));
          if (!success) {
            hppDout (info, "RHP stopped because it could not project a configuration");
            return false;
          }
          const Configuration_t q0 = path->initial ();
          const Configuration_t q2 = path->end ();
          // Velocities must be divided by two because each half is rescale
          // from [0, 0.5] to [0, 1]
          const vector_t vHalf = path->velocity (t) / 2;

          HermitePtr_t left = HPP_DYNAMIC_PTR_CAST(Hermite, steer (q0, q1));
          if (!left) throw std::runtime_error ("Not an path::Hermite");
          left->v0 (path->v0() / 2);
          left->v1 (vHalf);
          left->computeHermiteLength();

          HermitePtr_t right = HPP_DYNAMIC_PTR_CAST(Hermite, steer (q1, q2));
          if (!right) throw std::runtime_error ("Not an path::Hermite");
          right->v0 (vHalf);
          right->v1 (path->v1() / 2);
          right->computeHermiteLength();

          const value_type stopThr = beta_ * path->hermiteLength();
          bool lStop = ( left ->hermiteLength() > stopThr );
          bool rStop = ( right->hermiteLength() > stopThr );
          bool stop = rStop || lStop;
          // This is the inverse of the condition in the RSS paper. Is there a typo in the paper ?
          // if (std::max (left->hermiteLength(), right->hermiteLength()) > beta * path->hermiteLength()) {
          if (stop) {
            hppDout (info, "RHP stopped: " << path->hermiteLength() << " * " << beta_ << " -> " <<
                left->hermiteLength() << " / " << right->hermiteLength());
          }
          if (lStop || !recurse (left , proj, acceptThr)) return false;
          if ( stop || !recurse (right, proj, acceptThr)) return false;

          return true;
        }
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
