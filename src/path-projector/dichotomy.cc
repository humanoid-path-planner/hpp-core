// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include "hpp/core/path-projector/dichotomy.hh"

#include <queue>
#include <stack>
#include <stdexcept>

#include <hpp/core/path-vector.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    namespace pathProjector {
      Dichotomy::Dichotomy (const DistancePtr_t& distance,
			    const SteeringMethodPtr_t& steeringMethod,
			    value_type maxPathLength) :
        PathProjector (distance, steeringMethod), maxPathLength_ (maxPathLength)
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
        const Configuration_t& q1 = path->initial ();
        const Configuration_t& q2 = path->end ();
        if (cp) cp->rightHandSideFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        if (!cp || cp->dimension() == 0) {
          projection = path;
          return true;
        }

        bool pathIsFullyProjected = true;
        std::queue <PathPtr_t> paths;
        std::stack <PathPtr_t> pathToSplit;
        Configuration_t qi (q1.size());
        PathPtr_t sPath = steer (q1, q2);
        pathToSplit.push (sPath);
        while (!pathToSplit.empty ()) {
          sPath = pathToSplit.top ();
          pathToSplit.pop ();
          double l = sPath->length ();
          if (l < maxPathLength_) {
            paths.push (sPath);
            continue;
          }
          timeRange = sPath->timeRange ();
          const Configuration_t& qb = sPath->initial ();
          (*sPath) (qi, timeRange.first + l / 2);
          const Configuration_t& qe = sPath->end ();
          if (!constraints->apply (qi)) {
            pathIsFullyProjected = false;
            break;
          }
          PathPtr_t firstPart = steer (qb, qi);
          PathPtr_t secondPart = steer (qi, qe);
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
