
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

#include <boost/algorithm/string.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

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
      }

      ConfigOptimization::Parameters::Parameters () :
        addConfigConstraintToPath (ConfigOptimizationTraits::addConfigConstraintToPath ()),
        numberOfPass (ConfigOptimizationTraits::numberOfPass ()),
        numberOfIterations (ConfigOptimizationTraits::numberOfIterations ()),
        getGoal (ConfigOptimizationTraits::getGoal)
      {}

      ConfigOptimizationPtr_t ConfigOptimization::create (const Problem& problem)
      {
        return createWithTraits <ConfigOptimizationTraits> (problem);
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
        PathVectorPtr_t opted = unpacked;
        const std::size_t N = problem().robot()->configSize ();
        const std::size_t P = unpacked->numberPaths();

        Optimizers_t optimizers;
        const std::size_t rIndex = buildOptimizers (*unpacked, optimizers);
        vector_t configs (N*(P+1)), newConfigs;
        buildConfigVector (*unpacked, configs);
        newConfigs = configs;

        SteeringMethod& sm (*problem ().steeringMethod ());
        value_type length = pathLength (unpacked, problem().distance());
        hppDout (info, "ConfigOptimization: length " << length);
        value_type alpha = parameters.alphaInit;
        for (std::size_t ipass = 0; ipass < parameters.numberOfPass; ++ipass) {
          PathVectorPtr_t optedF = PathVector::create (path->outputSize(),
              path->outputDerivativeSize ());
          PathVectorPtr_t optedB = PathVector::create (path->outputSize(),
              path->outputDerivativeSize ());
          bool didChangeF, didChangeB;
          bool successF = pass <true> (configs, newConfigs, optimizers,
              rIndex  , alpha, optedF, didChangeF);
          if (successF) {
            bool successB = pass <false> (configs, newConfigs, optimizers,
                rIndex + 1, alpha, optedB, didChangeB);
            if (successB && (didChangeF || didChangeB)) {
              PathPtr_t link = sm (
                  newConfigs.segment ((rIndex    )*N, N),
                  newConfigs.segment ((rIndex + 1)*N, N));
              if (!link) {
                link = sm (
                    newConfigs.segment ((rIndex + 1)*N, N),
                    newConfigs.segment ((rIndex    )*N, N));
                if (link) link = link->reverse ();
              }
              if (isValid (link)) {
                opted = optedF;
                opted->appendPath (link);
                opted->concatenate (*optedB);
                /// OptedF is a valid optimization.
                value_type optedLength = pathLength (opted, problem().distance());
                if (optedLength < length) {
                  unpacked = opted;
                  length = optedLength;
                  hppDout (info, "ConfigOptimization: accepted length " << length);
                  /// Update configs
                  configs = newConfigs;
                  continue;
                }
              }
            }
          }
          /// Failure
          hppDout (info, "ConfigOptimization: pass " << ipass << " failed");
          alpha /= 2.;
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
        (const PathVector& path) const
      {
        std::vector <bool> mask (problem().robot()->numberDof(), true);
        // TODO: there are some DOF that should not be optimized using this
        // method. Here, we consider we do not want to optimized the root joint.
        // This is not a very good solution. For a freeflyer joint for instance,
        // you may want not to optimize the two first joints...
        const JointVector_t& jv = problem().robot()->getJointVector ();
        for (JointVector_t::const_iterator it = jv.begin ();
            it != jv.end (); ++it) {
          for (size_type i = 0; i < (*it)->numberDof (); i++)
            if (parameters.shouldFilter (*it, i))
              mask[(*it)->rankInVelocity () + i] = false;
        }
        Configuration_t goal = parameters.getGoal (path);
        NumericalConstraintPtr_t cc = NumericalConstraint::create
          (constraints::ConfigurationConstraint::create
           ("Optimization constraint", problem().robot(), goal, mask)
           );
        cc->function().context ("optimization");
        return cc;
      }

      bool ConfigOptimization::Optimizer::optimize (ConfigurationOut_t q,
          const std::size_t numIter,
          const value_type alpha) const
      {
        return proj->optimize (q, numIter, alpha);
      }

      ConfigProjectorPtr_t ConfigOptimizationTraits::getConfigProjector
        (const PathPtr_t& before, const PathPtr_t& /*after*/,
         bool& isReverse)
      {
        isReverse = false;
        ConfigProjectorPtr_t proj;
        ConstraintPtr_t c = before->constraints();
        if (c) {
          proj = HPP_DYNAMIC_PTR_CAST (ConfigProjector, c);
          if (!proj) {
            ConstraintSetPtr_t set =
              HPP_DYNAMIC_PTR_CAST (ConstraintSet, c);
            if (!set)
              throw std::runtime_error ("There are constraints of unknown types.");
            if (set->configProjector ()) {
              proj = set->configProjector ();
            }
          }
        }
        return proj;
      }

      std::size_t ConfigOptimization::buildOptimizers (const PathVector& pv,
          Optimizers_t& projectors)
      {
        NumericalConstraintPtr_t cc = createNumConstraint (pv);

        const std::size_t P = pv.numberPaths ();
        projectors.resize (P - 1);
        PathPtr_t cur = pv.pathAtRank (0);
        bool reverse = false, reverseStarted = false;
        std::size_t indexReverse = P-1;
        for (std::size_t i = 0; i < P - 1; ++i) {
          PathPtr_t next = pv.pathAtRank (i + 1);

          projectors[i].proj = parameters.getConfigProjector (cur, next, reverse);
          if (!projectors[i].proj) {
            projectors[i].proj = ConfigProjector::create ( problem().robot(),
                "Optimizer", 1e6, parameters.numberOfIterations);
          } else {
            projectors[i].proj = HPP_STATIC_PTR_CAST (ConfigProjector,
                projectors[i].proj->copy ());
          }
          // TODO: this is not robust since we do not know if level 1 is the
          // last
          projectors[i].proj->add (cc, SizeIntervals_t (0), 1);
          projectors[i].proj->lastIsOptional (true);

          if (reverse) {
            if (!reverseStarted) {
              indexReverse = i;
              reverseStarted = true;
            }
          }
          cur = next;
        }
        return indexReverse;
      }

      void ConfigOptimization::buildConfigVector (const PathVector& path,
          vectorOut_t configs) const
      {
        const std::size_t N = problem().robot()->configSize ();
        const std::size_t P = path.numberPaths();
        configs.resize (N * (P+1));

        std::size_t iP = 0;
        configs.segment (iP, N) = path.initial ();
        for (std::size_t i = 0; i < path.numberPaths(); ++i) {
          PathPtr_t cur = path.pathAtRank (i);
          iP += N;
          configs.segment (iP, N) = cur->end ();
        }
      }

      bool ConfigOptimizationTraits::shouldFilter (const JointPtr_t joint,
          const size_type /* iDof */)
      {
        // If joint if the root joint or if parent joint is the root joint and
        // root joint is an anchor joint,
        // filter it
        if (joint->parentJoint () == NULL ||
            (joint->parentJoint ()->parentJoint ()==NULL
             && joint->parentJoint()->numberDof () == 0)
           )
          return true;
        /// Filter joint name containing "base_joint..."
        if (boost::algorithm::contains (joint->name (), "base_joint"))
          return true;
        return false;
      }

      template <bool forward>
      bool ConfigOptimization::pass (
          vectorIn_t configs, vectorOut_t newConfigs,
          const Optimizers_t& optimizers, const std::size_t& index,
          const value_type& alpha, PathVectorPtr_t opted, bool& didChange
          ) const
      {
        SteeringMethod& sm (*problem ().steeringMethod ());
        std::list <PathPtr_t> paths;
        const std::size_t N = opted->outputSize ();
        const std::size_t P = optimizers.size () + 1;
        didChange = false;
        for (std::size_t i = (forward)? 1          : P-1;
                             (forward)? i <= index : i >= index;
                             (forward)? ++i        : --i) {
          newConfigs.segment (i * N, N) = configs.segment (i * N, N);
          std::size_t iN = (forward)? i - 1 : i + 1;
          /// Optimize
          bool isOpted = optimizers[i-1].optimize (
              newConfigs.segment (i * N, N),
              parameters.numberOfIterations, alpha);
          PathPtr_t current;
          if (isOpted)
            current = sm (newConfigs.segment (iN*N, N),
                          newConfigs.segment (i *N, N));
          if (!isOpted || !isValid (current)) {
            /// Try from last newConfigs to old ones.
            current = sm (newConfigs.segment (iN*N, N),
                             configs.segment (i *N, N));
            if (isValid (current)) {
              /// We do not change that configuration.
              newConfigs.segment (i * N, N) = configs.segment (i * N, N);
            } else {
              /// We should backtrack to previous iteration
              hppDout (info, "ConfigOptimization: failed at path rank " << i);
              return false;
            }
          } else
            didChange = true;
          if (forward) paths.push_back  (current);
          else         paths.push_front (current->reverse ());
        }

        /// Build the path
        for (std::list<PathPtr_t>::iterator it = paths.begin();
            it != paths.end (); ++it)
            opted->appendPath (*it);
        return true;
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
