
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

#include <hpp/core/path-optimization/config-optimization.hh>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/constraints/configuration-constraint.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/numerical-constraint.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      namespace {
        // Compute the length of a vector of paths assuming that each element
        // is optimal for the given distance.
        static value_type pathLength (const PathVectorPtr_t& path,
            const DistancePtr_t& distance)
        {
          value_type result = 0;
          for (std::size_t i=0; i<path->numberPaths (); ++i) {
            const PathPtr_t& element (path->pathAtRank (i));
            result += (*distance) (element->initial (), element->end ());
          }
          return result;
        }

        static value_type pathLength (const PathPtr_t& path,
            const DistancePtr_t& distance)
        {
          return (*distance) (path->initial (), path->end ());
        }
      }

      ConfigOptimization::Parameters::Parameters () :
        addConfigConstraintToPath (ConfigOptimizationTraits::addConfigConstraintToPath ()),
        numberOfPass (ConfigOptimizationTraits::numberOfPass ()),
        numberOfIterations (ConfigOptimizationTraits::numberOfIterations ()),
        getGoal (ConfigOptimizationTraits::getGoal)
      {}

      ConfigOptimizationPtr_t ConfigOptimization::create (const Problem& problem)
      {
        return createWithTrait <ConfigOptimizationTraits> (problem);
      }

      ConfigOptimization::ConfigOptimization (const Problem& problem) :
        PathOptimizer (problem)
      {
      }

      PathVectorPtr_t ConfigOptimization::optimize (const PathVectorPtr_t& path)
      {
        PathVectorPtr_t unpacked = PathVector::create (path->outputSize(),
            path->outputDerivativeSize ());
        path->flatten (unpacked);
        PathVectorPtr_t opted;

        /// Choose first configuration
        NumericalConstraintPtr_t cc = createNumConstraint (unpacked);

        /// Step 1: Try to optimize each waypoint
        Configuration_t qi, qm, qe;
        SteeringMethod& sm (*problem ().steeringMethod ());
        value_type length = pathLength (unpacked, problem().distance());
        hppDout (info, "ConfigOptimization: length " << length);
        for (std::size_t ipass = 0; ipass < parameters.numberOfPass; ++ipass) {
          value_type totalLength = 0;
          qi = unpacked->initial ();
          opted = PathVector::create (path->outputSize(),
              path->outputDerivativeSize ());
          PathPtr_t current = unpacked->pathAtRank (0);
          value_type currentL = pathLength (current, problem().distance());
          for (std::size_t i = 0; i < unpacked->numberPaths() - 1; ++i) {
            PathPtr_t next = unpacked->pathAtRank (i + 1);
            value_type nextL = pathLength (next, problem().distance());

            qm = next->initial ();
            qe = next->end ();
            ConfigProjectorPtr_t proj; 
            ConstraintPtr_t newSet;
            if (next->constraints ()) {
              newSet = next->constraints ()->copy ();
              proj = HPP_DYNAMIC_PTR_CAST (ConfigProjector, newSet);
              if (!proj) {
                ConstraintSetPtr_t set =
                  HPP_DYNAMIC_PTR_CAST (ConstraintSet, newSet);
                if (!set)
                  throw std::runtime_error ("There are constraints of unknown types.");
                if (!set->configProjector ()) {
                  proj = ConfigProjector::create ( problem().robot(),
                      "Optimizer", 1e6, parameters.numberOfIterations);
                  set->addConstraint (proj);
                } else {
                  proj = set->configProjector ();
                }
              }
            } else {
              proj = ConfigProjector::create ( problem().robot(),
                  "Optimizer", 1e6, parameters.numberOfIterations);
              newSet = proj;
            }
            // TODO: this is not robust since we do not know if level 1 is the
            // last
            proj->add (cc, SizeIntervals_t (0), 1);
            proj->lastIsOptional (true);

            if (!proj->optimize (qm, parameters.numberOfIterations)) {
              /// Nothing to do
              opted->appendPath (current);
              current = next;
              totalLength += currentL;
              currentL = nextL;
            } else {
              /// optimization worked. Check the paths.
              PathPtr_t newCurrent = sm (qi, qm);
              PathPtr_t newNext = sm (qm, qe);
              // TODO: The length will probably increase. I do not know if it
              // is relevant to check if the new path is shorter.
              // It might be better to only check the full path length in the
              // end.
              // if (nnL + ncL < currentL + nextL
              if (isValid (newCurrent) && isValid (newNext)) {
                value_type nnL = pathLength (newNext, problem().distance());
                value_type ncL = pathLength (newCurrent, problem().distance());
                opted->appendPath (newCurrent);
                current = newNext;
                totalLength += ncL;
                currentL = nnL;
              } else {
                opted->appendPath (current);
                current = next;
                totalLength += currentL;
                currentL = nextL;
              }
            }

            /// Prepare next iteration.
            qi = qm;
          }
          opted->appendPath (current);
          totalLength += currentL;
          if (totalLength < length) { // Success
            unpacked = opted;
            length = totalLength;
            hppDout (info, "ConfigOptimization: accepted length " << length);
          } else { // Fail
            opted = unpacked;
            hppDout (info, "ConfigOptimization: rejected length " << length);
            // length is unchanged.
          }
        }

        return opted;
      }

      bool ConfigOptimization::isValid (const PathPtr_t& p) const
      {
        PathPtr_t vp;
        PathValidationReportPtr_t r;
        if (!p) return false;
        else {
          return problem ().pathValidation ()->validate (p, false, vp, r);
        }
      }

      NumericalConstraintPtr_t ConfigOptimization::createNumConstraint
        (const PathVectorPtr_t& path) const
      {
        std::vector <bool> mask (problem().robot()->numberDof(), true);
        // TODO: there are some DOF that should not be optimized using this
        // method. Here, we consider we do not want to optimized the root joint.
        // This is not a very good solution. For a freeflyer joint for instance,
        // you may want not to optimize the two first joints...
        for (size_type i = 0;
            i < problem().robot()->rootJoint ()->numberDof (); i++)
          mask[i] = false;
        Configuration_t goal = parameters.getGoal (*path);
        NumericalConstraintPtr_t cc = NumericalConstraint::create
          (constraints::ConfigurationConstraint::create
           ("Optimization constraint", problem().robot(), goal, mask)
           );
        cc->function().context ("optimization");
        return cc;
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
