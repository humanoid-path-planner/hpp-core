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

#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
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
      namespace {
        HPP_DEFINE_TIMECOUNTER (globalPathProjector_initCfgList);
        HPP_DEFINE_TIMECOUNTER (globalPathProjector_projOneStep);
        HPP_DEFINE_TIMECOUNTER (globalPathProjector_reinterpolate);
        HPP_DEFINE_TIMECOUNTER (globalPathProjector_createPath);
      }
      Global::Global (const DistancePtr_t& distance,
				const SteeringMethodPtr_t& steeringMethod,
				value_type step) :
        PathProjector (distance, steeringMethod), step_ (step),
        alphaMin (0.2), alphaMax (0.95)
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
        HPP_START_TIMECOUNTER(globalPathProjector_initCfgList);
        initialConfigList (path, cfgs);
        if (cfgs.size() == 2) { // Shorter than step_
          proj = path;
          return true;
        }
        assert ((cfgs.back () - path->end ()).isZero ());
        HPP_STOP_AND_DISPLAY_TIMECOUNTER(globalPathProjector_initCfgList);

        Bools_t projected (cfgs.size () - 2, false);
        Alphas_t alphas   (cfgs.size () - 2, alphaMin);
        Lengths_t lengths (cfgs.size () - 1, 0);
        Configs_t::iterator last = --(cfgs.end());

        std::size_t nbIter = 0;
        const std::size_t maxIter = p.maxIterations ();
        const std::size_t maxCfgNum =
          2 + (std::size_t)(10 * path->length() / step_);

        hppDout (info, "start with " << cfgs.size () << " configs");
        while (!projectOneStep (p, cfgs, last, projected, lengths, alphas)) {
          assert ((cfgs.back () - path->end ()).isZero ());
          if (cfgs.size() < maxCfgNum) {
            const size_type newCs =
              reinterpolate (p.robot(), cfgs, last, projected, lengths, alphas,
                  step_);
            if (newCs > 0) {
              nbIter = 0;
              hppDout (info, "Added " << newCs << " configs. Cur / Max = "
                  << cfgs.size() << '/' << maxCfgNum);
            } else if (newCs < 0) {
              hppDout (info, "Removed " << -newCs << " configs. Cur / Max = "
                  << cfgs.size() << '/' << maxCfgNum);
            }
          }

          nbIter++;

          if (nbIter > maxIter) break;
        }
        HPP_DISPLAY_TIMECOUNTER(globalPathProjector_projOneStep);
        HPP_DISPLAY_TIMECOUNTER(globalPathProjector_reinterpolate);

        // Build the projection
        HPP_START_TIMECOUNTER (globalPathProjector_createPath);
        bool ret = createPath (p.robot(), path->constraints(),
            cfgs, last, projected, lengths, proj);
        HPP_STOP_AND_DISPLAY_TIMECOUNTER (globalPathProjector_createPath);
        return ret;
      }

      bool Global::projectOneStep (ConfigProjector& p,
          Configs_t& q, Configs_t::iterator& last,
          Bools_t& b, Lengths_t& l, Alphas_t& a) const
      {
        HPP_START_TIMECOUNTER(globalPathProjector_projOneStep);
        /// First and last should not be updated
        const Configs_t::iterator begin = ++(q.begin ());
        const Configs_t::iterator end   = --(q.end ());
        Bools_t  ::iterator itB   =   (b.begin ());
        Alphas_t ::iterator itA   =   (a.begin ());
        Lengths_t::iterator itL   =   (l.begin ());
        Configs_t::iterator itCp  =   (q.begin ());
        bool allAreSatisfied = true;
        bool curUpdated = false, prevUpdated = false;
        /// Eigen matrices storage order defaults to column major.
        Eigen::Matrix <value_type, Eigen::Dynamic, 2>
          oldQ (p.robot()->configSize(),2);
        Eigen::Matrix <value_type, Eigen::Dynamic, 2>
          dq (p.robot()->numberDof(),2);
        dq.setZero();
        vector_t qMinusQPrev (p.robot()->numberDof());
        size_type iCol = 0;
        size_type iNCol = (iCol+1)%2;
        for (Configs_t::iterator it = begin; it != last; ++it) {
          if (!*itB) {
            oldQ.col(iCol) = *it;
            *itB = p.oneStep (*it, dq.col(iCol),*itA);
            *itA = alphaMax - 0.8 * (alphaMax - *itA);
            allAreSatisfied = allAreSatisfied && *itB;
            curUpdated = true;
            if (prevUpdated) {

              /// Detect large increase in size
              hpp::model::difference (p.robot(), oldQ.col(iCol), oldQ.col(iNCol), qMinusQPrev);
              const vector_t deltaDQ = dq.col(iCol) - dq.col(iNCol);
              const value_type N2 = qMinusQPrev.squaredNorm();
              if (sqrt(N2) < Eigen::NumTraits<value_type>::dummy_precision ()) {
                hppDout (error, "The two config should be removed:"
                    << "\noldQ = " << oldQ.transpose()
                    << "\nqMinusQPrev = " << qMinusQPrev.transpose()
                    << "\nDistance is " << d (oldQ.col(iCol), oldQ.col(iNCol))
                    << "\nand in mem  " << *itL
                    );
              } else {
                const value_type alphaSquare = 1 - 2 * qMinusQPrev.dot(deltaDQ) / N2 + qMinusQPrev.squaredNorm() / N2;
                // alpha > 4 => the distance between the two points has been
                // mutiplied by more than 4.
                if (alphaSquare > 16) {
                  hppDout (error, "alpha^2 = " << alphaSquare
                      << "\nqMinusQPrev = " << qMinusQPrev.transpose()
                      << "\ndeltaDQ = " << deltaDQ.transpose());
                  last = it; --last;
                  return false;
                }
              }

              const value_type limitCos = 0.5;
              vector_t dots = dq.colwise().normalized ().transpose() * qMinusQPrev.normalized();
              // Check if both updates are pointing outward
              if (dots[iCol] > limitCos && dots[iNCol] < - limitCos) {
                hppDout (error, "Descent step is going in opposite direction: "
                    << dots << ". It is likely a discontinuity.");
                  last = it; --last;
                  return false;
              }
              iNCol = iCol;
              iCol = (iCol+1)%2;
            }
          }
          if (prevUpdated || curUpdated)
            *itL = d (*itCp, *it);
          prevUpdated = curUpdated;
          curUpdated = false;
          ++itCp;
          ++itB;
          ++itL;
        }
        if (prevUpdated)
          *itL = d (*itCp, *end);
        HPP_STOP_TIMECOUNTER(globalPathProjector_projOneStep);
        return allAreSatisfied;
      }

      size_type Global::reinterpolate (const DevicePtr_t& robot,
          Configs_t& q, const Configs_t::iterator& last,
          Bools_t& b, Lengths_t& l, Alphas_t& a,
          const value_type& maxDist) const
      {
        HPP_START_TIMECOUNTER(globalPathProjector_reinterpolate);
        Configs_t::iterator begin = ++(q.begin ());
        Configs_t::iterator end   = last; ++end;
        Bools_t  ::iterator itB   =   (b.begin ());
        Alphas_t ::iterator itA   =   (a.begin ());
        Lengths_t::iterator itL   =   (l.begin ());
        Configs_t::iterator itCp  =   (q.begin ());
        Configuration_t newQ (robot->configSize ());
        size_type nbNewC = 0;
        for (Configs_t::iterator it = begin; it != last; ++it) {
          if (*itL > maxDist) {
            ++nbNewC;
            hpp::model::interpolate (robot, *itCp, *it, 0.5, newQ);
            // FIXME: make sure the iterator are valid after insertion
            // Insert new respective elements
            it  = q.insert (it, newQ);
            itB = b.insert (itB, false);
            itA = a.insert (itA, alphaMin);
            itL = l.insert (itL, d (*itCp, *it));
            // Update length after
            Configs_t::iterator itNC = it;  ++itNC;
            Lengths_t::iterator itNL = itL; ++itNL;
            *itNL = d (*it, *itNC);
            // FIXME: End has changed ?
            end = q.end();
            it = itCp;
            continue;
          } else if (*itL < maxDist * 1e-2) {
            nbNewC--;
            hppDout (warning, "Removing configuration: " << it->transpose()
                << "\nToo close to: " << itCp->transpose()
                );
            // The distance to the previous point is very small.
            // This point can safely be removed.
            it  = q.erase (it);
            itB = b.erase (itB);
            itA = a.erase (itA);
            itL = l.erase (itL);
            // Update length
            *itL = d (*itCp, *it);
            it = itCp;
            continue;
          }
          ++itCp;
          ++itB;
          ++itA;
          ++itL;
        }
        HPP_STOP_TIMECOUNTER(globalPathProjector_reinterpolate);
        return nbNewC;
      }

      bool Global::createPath (const DevicePtr_t& robot,
          const ConstraintSetPtr_t& constraint,
          const Configs_t& q, const Configs_t::iterator&,
          const Bools_t& b, const Lengths_t& l, PathPtr_t& result) const
      {
        /// Compute total length
        value_type length = 0;
        Lengths_t::const_iterator itL   = (l.begin ());
        Bools_t  ::const_iterator itB   = (b.begin ());
        Configs_t::const_iterator itCl  = (q.begin ());
        bool fullyProjected = true;
        for (; itB != b.end (); ++itB) {
          if (!*itB || *itL > step_) {
            fullyProjected = false;
            break;
          }
          length += *itL;
          ++itCl;
          ++itL;
        }
        if (fullyProjected) {
          length += *itL;
          ++itCl;
          ++itL;
          assert (itL == l.end ());
        }

        InterpolatedPathPtr_t out = InterpolatedPath::create
          (robot, q.front(), *itCl, length, constraint);

        if (itCl != q.begin ()) {
          length = 0;
          Lengths_t::const_iterator itL   =   (l.begin ());
          Configs_t::const_iterator begin = ++(q.begin ());
          for (Configs_t::const_iterator it = begin; it != itCl; ++it) {
            length += *itL;
            out->insert (length, *it);
            ++itL;
          }
        } else {
          hppDout (info, "Path of length 0");
          assert (!fullyProjected);
        }
        result = out;
        hppDout (info, "Projection succeeded ? " << fullyProjected);
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
          // Factor 0.99 is to ensure that the distance between two consecutives
          // configurations will be smaller that step_
          for (value_type t = step_; t < L; t += step_*0.99) {
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
