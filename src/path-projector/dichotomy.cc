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

      bool Dichotomy::impl_apply (const StraightPathPtr_t path, PathPtr_t& projection) const
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

        bool pathIsFullyProjected = true;
        std::queue <core::StraightPathPtr_t> paths;
        std::stack <core::StraightPathPtr_t> pathToSplit;
        Configuration_t qi (q1.size());
        core::StraightPathPtr_t sPath = core::StraightPath::create (sp.device (), q1, q2, d (q1, q2));
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
            core::StraightPath::create (sp.device (), qb, qi, d (qb, qi));
          StraightPathPtr_t secondPart =
            core::StraightPath::create (sp.device (), qi, qe, d (qi, qe));
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
            projection = paths.front ();
            projection->constraints (constraints);
            break;
          default:
            core::PathVectorPtr_t pv = core::PathVector::create (sp.outputSize (), sp.outputDerivativeSize ());
            while (!paths.empty ()) {
              paths.front ()->constraints (constraints);
              pv->appendPath (paths.front ());
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
