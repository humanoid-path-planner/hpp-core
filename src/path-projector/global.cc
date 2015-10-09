// Copyright (c) 2015, Joseph Mirabel
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

#include "hpp/core/path-projector/global.hh"

#include <hpp/model/configuration.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/config-projector.hh>

#include <limits>
#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      Global::Global (const DistancePtr_t& distance,
				const SteeringMethodPtr_t& steeringMethod,
				value_type step) :
        PathProjector (distance, steeringMethod), step_ (step)
      {}

      bool Global::impl_apply (const PathPtr_t& path,
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

      bool Global::project (const PathPtr_t& path, PathPtr_t& proj) const
      {
        Configs_t cfgs;
        ConfigProjector& p = *path->constraints ()->configProjector ();
        initialConfigList (path, cfgs);

        Bools_t projected (cfgs.size () - 2, false);
        Lengths_t lengths (cfgs.size () - 1, 0);

        value_type alpha = 0.2;
        const value_type alphaMax = 0.95;
        std::size_t nbIter = 0;
        const std::size_t maxIter = p.maxIterations ();

        while (projectOneStep (p, cfgs, projected, lengths, alpha)) {
          reinterpolate (p.robot(), cfgs, projected, lengths, step_);
          nbIter++;

          if (nbIter > maxIter) break;

          /// Prepare next iteration
          alpha = alphaMax - .8*(alphaMax - alpha);
        }

        // Build the projection
        return createPath (p.robot(), path->constraints(),
            cfgs, projected, lengths, proj);
      }

      bool Global::projectOneStep (ConfigProjector& p,
          Configs_t& q, Bools_t& b, Lengths_t& l,
          const value_type& alpha) const
      {
        /// First and last should not be updated
        const Configs_t::iterator begin = ++(q.begin ());
        const Configs_t::iterator end   = --(q.end ());
        Bools_t  ::iterator itB   =   (b.begin ());
        Lengths_t::iterator itL   =   (l.begin ());
        Configs_t::iterator itCp  =   (q.begin ());
        bool allAreSatisfied = true;
        bool curUpdated = false, prevUpdated = false;
        for (Configs_t::iterator it = begin; it != end; ++it) {
          if (!*itB) {
            *itB = p.oneStep (*it, alpha);
            allAreSatisfied = allAreSatisfied && *itB;
            curUpdated = true;
          }
          if (prevUpdated || curUpdated)
            *itL = d (*itCp, *it);
          prevUpdated = curUpdated;
          curUpdated = false;
          ++itCp;
          ++itB;
          ++itL;
        }
        return allAreSatisfied;
      }

      std::size_t Global::reinterpolate (const DevicePtr_t& robot,
          Configs_t& q, Bools_t& b, Lengths_t& l,
          const value_type& maxDist) const
      {
        const Configs_t::iterator begin = ++(q.begin ());
        Configs_t::iterator end   =   (q.end ());
        Bools_t  ::iterator itB   =   (b.begin ());
        Lengths_t::iterator itL   =   (l.begin ());
        Configs_t::iterator itCp  =   (q.begin ());
        Configuration_t newQ (robot->configSize ());
        std::size_t nbNewC = 0;
        for (Configs_t::iterator it = begin; it != end; ++it) {
          if (*itL > maxDist) {
            ++nbNewC;
            hpp::model::interpolate (robot, *itCp, *it, 0.5, newQ);
            // FIXME: make sure the iterator are valid after insertion
            // Insert new respective elements
            it = q.insert (it, newQ);
            itB = b.insert (itB, false);
            itL = l.insert (itL, d (*itCp, *it));
            // Update length after
            Configs_t::iterator itNC = it;  ++itNC;
            Lengths_t::iterator itNL = itL; ++itNL;
            *itNL = d (*it, *itNC);
            // FIXME: End has changed ?
            end = q.end();
            continue;
          }
          ++itCp;
          ++itB;
          ++itL;
        }
        return nbNewC;
      }

      bool Global::createPath (const DevicePtr_t& robot,
          ConstraintSetPtr_t constraint, Configs_t& q, Bools_t& b, Lengths_t& l,
          PathPtr_t& result)
      {
        /// Compute total length
        value_type length = 0;
        Bools_t  ::iterator itB   =   (b.begin ());
        Configs_t::iterator itCl  =   (q.begin ());
        bool fullyProjected = true;
        for (Lengths_t::iterator itL = l.begin (); itL != l.end (); ++itL) {
          if (!*itB) {
            fullyProjected = false;
            break;
          }
          ++itCl;
          ++itB;
          length += *itL;
        }
        if (fullyProjected) ++itCl;

        InterpolatedPathPtr_t out = InterpolatedPath::create
          (robot, q.front(), *itCl, length, constraint);

        Lengths_t::iterator itL   =   (l.begin ());
        Configs_t::iterator itCp  =   (q.begin ());

        length = 0;
        const Configs_t::iterator begin = ++(q.begin ());
        for (Configs_t::iterator it = begin; it != itCl; ++it) {
          length += *itL;
          out->insert (length, *it);
        }
        result = out;
        return fullyProjected;
      }

      void Global::initialConfigList (const PathPtr_t& path,
          Configs_t& cfgs) const
      {
        InterpolatedPathPtr_t ip =
          HPP_DYNAMIC_PTR_CAST (InterpolatedPath, path);
        if (ip) {
          // Get the waypoint of ip
          const InterpolatedPath::InterpolationPoints_t& ips =
            ip->interpolationPoints();
          for (InterpolatedPath::InterpolationPoints_t::const_iterator it =
              ips.begin (); it != ips.end (); ++it) {
            cfgs.push_back (it->second);
          }
        } else {
          const value_type L = path->length ();
          Configuration_t q (path->outputSize ());
          cfgs.push_back (path->initial ());
          for (value_type t = step_; t < L; t += step_) {
            // Interpolate without taking care of the constraints
            // FIXME: Path must not be a PathVector otherwise the constraints
            // are applied.
            path->at (t, q);
            cfgs.push_back (q);
          }
          cfgs.push_back (path->end ());
        }
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
