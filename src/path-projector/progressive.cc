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

      bool Progressive::impl_apply (const StraightPathPtr_t path, PathPtr_t& projection) const
      {
        ConstraintSetPtr_t constraints = path->constraints ();
        const ConfigProjectorPtr_t& cp = constraints->configProjector ();
        const StraightPath& sp = *path;
        core::interval_t timeRange = sp.timeRange ();
        const Configuration_t& q1 = sp(timeRange.first);
        const Configuration_t& q2 = sp(timeRange.second);
        if (cp) cp->rightHandSideFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        if (!cp) {
          projection = path;
          return true;
        }

        bool pathIsFullyProjected = false;
        std::queue <core::StraightPathPtr_t> paths;
        StraightPathPtr_t toSplit =
            core::StraightPath::create (sp.device (), q1, q2, d (q1, q2));
        Configuration_t qi (q1.size());
        value_type curStep, curLength;
        while (true) {
          const StraightPath& toSplitRef = *toSplit;
          if (toSplitRef.length () < step_) {
            paths.push (toSplit);
            pathIsFullyProjected = true;
            break;
          }
          timeRange = toSplitRef.timeRange ();
          const Configuration_t& qb = toSplitRef (timeRange.first);
          curStep = step_;
          curLength = std::numeric_limits <value_type>::max();
          bool stop = false;
          /// Find the good length.
          /// Here, it would be good to have an upper bound of the Hessian
          /// of the constraint.
          do {
            if (curStep < 1e-2) {
              stop = true;
              break;
            }
            toSplitRef (qi, curStep);
            if (constraints->apply (qi)) {
              curLength = d (qb, qi);
            }
            curStep /= 2;
          } while (curLength > step_ ||
              curLength < 1e-2);
          if (stop) break;
          StraightPathPtr_t part =
            core::StraightPath::create (sp.device (), qb, qi, curLength);
          paths.push (part);
          toSplit =
            core::StraightPath::create (sp.device (), qi, q2, d (qi, q2));
        }
        switch (paths.size ()) {
          case 0:
            timeRange = sp.timeRange();
            projection = sp.extract (std::make_pair (timeRange.first, timeRange.first));
            return false;
            break;
          case 1:
            projection = paths.front ();
            projection->constraints (constraints);
            break;
          default:
            core::PathVectorPtr_t pv = core::PathVector::create (sp.outputSize (), sp.outputDerivativeSize ());
            qi = q1;
            while (!paths.empty ()) {
              assert ((qi - (*paths.front ())(paths.front ()->timeRange().first)).isZero ());
              assert (constraints->isSatisfied (qi));
              qi = (*paths.front ())(paths.front ()->timeRange().second);
              assert (constraints->isSatisfied (qi));
              paths.front ()->constraints (constraints);
              pv->appendPath (paths.front ());
              paths.pop ();
            }
            projection = pv;
            break;
        }
        assert (((*projection)(projection->timeRange ().first) - (*path)(path->timeRange ().first)).isZero());
        assert (!pathIsFullyProjected || ((*projection)(projection->timeRange ().second) - (*path)(path->timeRange ().second)).isZero());
        return pathIsFullyProjected;
      }
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
