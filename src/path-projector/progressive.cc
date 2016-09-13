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

#include <hpp/util/timer.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/config-projector.hh>

// TODO used to access parameters of the problem. We should not do this here.
//      See todo of pathProjector::Global::create
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/problem.hh>

#include <limits>
#include <queue>
#include <stack>

namespace hpp {
  namespace core {
    namespace pathProjector {
      ProgressivePtr_t Progressive::create (const DistancePtr_t& distance,
          const SteeringMethodPtr_t& steeringMethod, value_type step)
      {
        value_type hessianBound = -1;
        value_type thr_min = 1e-3;
        try {
          hessianBound = steeringMethod->problem()->getParameter<value_type>
            ("PathProjectionHessianBound", hessianBound);
          thr_min = steeringMethod->problem()->getParameter<value_type>
            ("PathProjectionMinimalDist", thr_min);
          hppDout (info, "Hessian bound is " << hessianBound);
          hppDout (info, "Min Dist is " << thr_min);
        } catch (const boost::bad_any_cast& e) {
          hppDout (error, "Could not cast parameter "
              "PathProjectionHessianBound or PathProjectionMinimalDist "
              "to value_type");
        }
        return ProgressivePtr_t (new Progressive (distance, steeringMethod,
              step, thr_min, hessianBound));
      }

      ProgressivePtr_t Progressive::create (const ProblemPtr_t& problem,
          const value_type& step)
      {
        value_type hessianBound = -1;
        value_type thr_min = 1e-3;
        try {
          hessianBound = problem->getParameter<value_type>
            ("PathProjectionHessianBound", hessianBound);
          thr_min = problem->getParameter<value_type>
            ("PathProjectionMinimalDist", thr_min);
          hppDout (info, "Hessian bound is " << hessianBound);
          hppDout (info, "Min Dist is " << thr_min);
        } catch (const boost::bad_any_cast& e) {
          hppDout (error, "Could not cast parameter "
              "PathProjectionHessianBound or PathProjectionMinimalDist "
              "to value_type");
        }
        return ProgressivePtr_t (new Progressive (problem->distance(),
              problem->steeringMethod(),
              step, thr_min, hessianBound));
      }

      Progressive::Progressive (const DistancePtr_t& distance,
				const SteeringMethodPtr_t& steeringMethod,
				value_type step, value_type thresholdMin, value_type hessianBound) :
        PathProjector (distance, steeringMethod), step_ (step),
        thresholdMin_ (thresholdMin),
        hessianBound_ (hessianBound), withHessianBound_ (hessianBound > 0)
      {
        // TODO Only SteeringMethodStraight has been tested so far.
        assert (HPP_DYNAMIC_PTR_CAST(hpp::core::SteeringMethodStraight, steeringMethod));
      }

      bool Progressive::impl_apply (const PathPtr_t& path,
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

      bool Progressive::project (const PathPtr_t& path, PathPtr_t& proj) const
      {
        ConstraintSetPtr_t constraints = path->constraints ();
	if (!constraints) {
	  proj = path;
	  return true;
	}
        const ConfigProjectorPtr_t& cp = constraints->configProjector ();
        core::interval_t timeRange = path->timeRange ();
        const Configuration_t& q1 = path->initial ();
        const Configuration_t& q2 = path->end ();
        Configuration_t qtmp = q1;
        vector_t dqtmp (vector_t::Zero(cp->robot()->numberDof()));
        const size_t maxDichotomyTries = 10,
                     maxPathSplit =
	  (size_t)(10 * (timeRange.second - timeRange.first) / (double)step_);
	assert (constraints->isSatisfied (q1));
        if (!constraints->isSatisfied (q2)) return false;
        if (!cp) {
          proj = path;
          return true;
        }

        bool pathIsFullyProjected = false;
        std::queue <PathPtr_t> paths;
        PathPtr_t toSplit = steer (q1, q2);
        Configuration_t qi (q1.size());
        value_type curStep, curLength, totalLength = 0;
        size_t c = 0;
        const value_type& K = hessianBound_; // upper bound of Hessian
        if (withHessianBound_) cp->oneStep (qtmp, dqtmp, 1);
        value_type sigma = cp->sigma();

        value_type min = std::numeric_limits<value_type>::max(), max = 0;

        while (true) {
          const value_type threshold = (withHessianBound_ ? sigma / K : step_);
          const value_type thr_min = thresholdMin_;

          if (toSplit->length () < threshold) {
            paths.push (toSplit);
            totalLength += toSplit->length ();
            pathIsFullyProjected = true;
            break;
          }
          const Configuration_t& qb = toSplit->initial ();
          curLength = std::numeric_limits <value_type>::max();
          size_t dicC = 0;

          curStep = threshold - Eigen::NumTraits<value_type>::epsilon();

          /* if (withHessianBound_) */ if (threshold < thr_min) break;
          // if (withHessianBound_) std::cout << threshold << std::endl;

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
	  assert (curLength == d (qb, qi));

          if (withHessianBound_) {
            /// Update sigma
            qtmp = qi;
            cp->oneStep (qtmp, dqtmp, 1);
            sigma = cp->sigma();
          }

          PathPtr_t part = steer (qb, qi);
          paths.push (part);
          totalLength += part->length ();
          min = std::min(min, part->length());
          max = std::max(max, part->length());
          toSplit = steer (qi, q2);
          c++;
        }
#if HPP_ENABLE_BENCHMARK
        hppBenchmark("Interpolated path (progressive): "
            << paths.size()
            << ", [ " << min
            <<   ", " << (paths.empty() ? 0 : totalLength / paths.size())
            <<   ", " << max << "]"
            );
#endif
        switch (paths.size ()) {
          case 0:
            timeRange = path->timeRange();
            proj = path->extract
	      (std::make_pair (timeRange.first, timeRange.first));
            return false;
            break;
          case 1:
            proj = paths.front ()->copy (constraints);
            break;
          default:
            InterpolatedPathPtr_t p = InterpolatedPath::create
              (cp->robot (), q1, paths.back()->end(), totalLength,
               path->constraints ());
            value_type t = paths.front ()->length ();
            qi = paths.front()->end ();
            paths.pop ();
            while (!paths.empty ()) {
              assert ((qi - paths.front ()->initial ()).isZero ());
              qi = paths.front ()->end ();
              p->insert (t, paths.front()->initial ());
              assert (t <= totalLength);
              t += paths.front()->length ();
              paths.pop ();
            }
            proj = p;
            break;
        }
	if (d (proj->initial (), path->initial ()) != 0) {
	  hppDout (error, "proj->initial () = "
		   << proj->initial ().transpose ());
	  hppDout (error, "path->initial ()       = "
		   << path->initial ().transpose ());
	}
        assert (d (proj->initial (), path->initial ()) == 0);
	if (pathIsFullyProjected &&
	    (d (proj->end (), path->end ()) != 0)) {
	  hppDout (error, "proj->end () = "
		   << proj->end ().transpose ());
	  hppDout (error, "path->end ()       = " << path->end ().transpose ());
	  hppDout (error, "d (proj->end (), path->end ()) = "
		   << d (proj->end (), path->end ()));
	  hppDout (error, "proj->end () - path->end () = "
		   << (proj->end () - path->end ()).transpose ());
	}
        assert (!pathIsFullyProjected ||
		(d (proj->end (), path->end ()) == 0));
        return pathIsFullyProjected;
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
