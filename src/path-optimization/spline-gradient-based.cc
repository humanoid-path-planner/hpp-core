// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/core/path-optimization/spline-gradient-based.hh>

#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>

#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/collision-path-validation-report.hh>

#include <path-optimization/spline-gradient-based/cost.hh>
#include <path-optimization/spline-gradient-based/collision-constraint.hh>
#include <path-optimization/spline-gradient-based/eiquadprog_2011.hpp>

namespace hpp {
  namespace core {
    using pinocchio::Device;

    namespace pathOptimization {
      typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix_t;
      typedef Eigen::Map<const vector_t> ConstVectorMap_t;
      typedef Eigen::Map<      vector_t>      VectorMap_t;

      typedef Eigen::BlockIndex BlockIndex;

      HPP_DEFINE_TIMECOUNTER(SGB_constraintDecomposition);
      HPP_DEFINE_TIMECOUNTER(SGB_qpDecomposition);
      HPP_DEFINE_TIMECOUNTER(SGB_findNewConstraint);
      HPP_DEFINE_TIMECOUNTER(SGB_qpSolve);

      template <int NbRows>
      VectorMap_t reshape (Eigen::Matrix<value_type, NbRows, Eigen::Dynamic, Eigen::RowMajor>& parameters)
      {
        return VectorMap_t (parameters.data(), parameters.size());
      }

      template <int NbRows>
      ConstVectorMap_t reshape (const Eigen::Matrix<value_type, NbRows, Eigen::Dynamic, Eigen::RowMajor>& parameters)
      {
        return ConstVectorMap_t (parameters.data(), parameters.size());
      }

      template <int _PB, int _SO>
      SplineGradientBased<_PB, _SO>::SplineGradientBased (const Problem& problem)
        : Base (problem)
        , checkOptimum_ (false)
      {}

      // ----------- Convenience class -------------------------------------- //

      /** \f{eqnarray*}{
       *  min & 0.5 * x^T H x + b^T x \\
       *      & lc.J * x = lc.b
       *  \f}
      **/
      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::QuadraticProblem
      {
        typedef Eigen::JacobiSVD < matrix_t > Decomposition_t;
        typedef Eigen::LLT <matrix_t, Eigen::Lower> LLT_t;

        QuadraticProblem (size_type inputSize) :
          H (inputSize, inputSize), b (inputSize),
          dec (inputSize, inputSize, Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (inputSize)
        {
          H.setZero();
          b.setZero();
          bIsZero = true;
        }

        QuadraticProblem (const QuadraticProblem& QP, const LinearConstraint& lc) :
          H (lc.PK.cols(), lc.PK.cols()), b (lc.PK.cols()), bIsZero (false),
          dec (lc.PK.cols(), lc.PK.cols(), Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (lc.PK.cols())
        {
          QP.reduced (lc, *this);
        }

        QuadraticProblem (const QuadraticProblem& QP) :
          H (QP.H), b (QP.b), bIsZero (QP.bIsZero),
          dec (QP.dec), xStar (QP.xStar)
        {}

        void addRows (const std::size_t& nbRows)
        {
          H.conservativeResize(H.rows() + nbRows, H.cols());
          b.conservativeResize(b.rows() + nbRows, b.cols());

          H.bottomRows(nbRows).setZero();
        }

        /*/ Compute the problem
         *  \f{eqnarray*}{
         *  min & 0.5 * x^T H x + b^T x \\
         *      & lc.J * x = lc.b
         *  \f}
        **/
        void reduced (const LinearConstraint& lc, QuadraticProblem& QPr) const
        {
          matrix_t H_PK (H * lc.PK);
          QPr.H.noalias() = lc.PK.transpose() * H_PK;
          QPr.b.noalias() = H_PK.transpose() * lc.xStar;
          if (!bIsZero) {
            QPr.b.noalias() += lc.PK.transpose() * b;
          }
          QPr.bIsZero = false;
        }

        void decompose ()
        {
          HPP_SCOPE_TIMECOUNTER(SGB_qpDecomposition);
          dec.compute(H);
          assert(dec.rank() == H.rows());
        }

        void solve ()
        {
          xStar.noalias() = - dec.solve(b);
        }

        void computeLLT()
        {
          HPP_SCOPE_TIMECOUNTER(SGB_qpDecomposition);
          trace = H.trace();
          llt.compute(H);
        }

        double solve(const LinearConstraint& ce, const LinearConstraint& ci)
        {
          HPP_SCOPE_TIMECOUNTER(SGB_qpSolve);
          // min   0.5 * x G x + g0 x
          // s.t.  CE^T x + ce0 = 0
          //       CI^T x + ci0 >= 0
          return solve_quadprog2 (llt, trace, b,
              ce.J.transpose(), - ce.b,
              ci.J.transpose(), - ci.b,
              xStar, activeConstraint, activeSetSize);
        }

        // model
        matrix_t H;
        vector_t b;
        bool bIsZero;

        // Data
        LLT_t llt;
        value_type trace;
        Eigen::VectorXi activeConstraint;
        int activeSetSize;

        // Data
        Decomposition_t dec;
        vector_t xStar;
      };

      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::CollisionFunctions
      {
        void addConstraint (const CollisionFunctionPtr_t& f,
                            const std::size_t& idx,
                            const size_type& row,
                            const value_type& r)
        {
          assert (f->outputSize() == 1);
          functions.push_back(f);
          splineIds.push_back(idx);
          rows.push_back(row);
          ratios.push_back(r);
        }

        void removeLastConstraint (const std::size_t& n, LinearConstraint& lc)
        {
          assert (functions.size() >= n && std::size_t(lc.J.rows()) >= n);

          const std::size_t nSize = functions.size() - n;
          functions.resize(nSize);
          splineIds.resize(nSize);
          rows.resize(nSize);
          ratios.resize(nSize);

          lc.J.conservativeResize(lc.J.rows() - n, lc.J.cols());
          lc.b.conservativeResize(lc.b.rows() - n, lc.b.cols());
        }

        // Compute linearization
        // b = f(S(t))
        // J = Jf(S(p, t)) * dS/dp
        // f(S(t)) = b -> J * P = b
        void linearize (const SplinePtr_t& spline, const SplineOptimizationData& sod,
            const std::size_t& fIdx, LinearConstraint& lc)
        {
          const CollisionFunctionPtr_t& f = functions[fIdx];

          const size_type row = rows[fIdx],
                          nbRows = 1,
                          rDof = f->inputDerivativeSize();
          const value_type t = spline->length() * ratios[fIdx];

          q.resize(f->inputSize());
          (*spline) (q, t);

          // Evaluate explicit functions
          if (sod.es) sod.es->solve(q);

          LiegroupElement v (f->outputSpace());
          f->value(v, q);

          J.resize(f->outputSize(), f->inputDerivativeSize());
          f->jacobian(J, q);

          // Apply chain rule if necessary
          if (sod.es) {
            Js.resize(sod.es->derSize(), sod.es->derSize());
            sod.es->jacobian(Js, q);

            sod.es->inDers().lview(J) =
              sod.es->inDers().lview(J).eval() +
              sod.es->outDers().transpose().rview(J).eval()
              * sod.es->viewJacobian(Js).eval(); 
            sod.es->outDers().transpose().lview(J).setZero();
          }

          spline->parameterDerivativeCoefficients(paramDerivativeCoeff, t);

          const size_type col = splineIds[fIdx] * Spline::NbCoeffs * rDof;
          for (size_type i = 0; i < Spline::NbCoeffs; ++i)
            lc.J.block (row, col + i * rDof, nbRows, rDof).noalias()
              = paramDerivativeCoeff(i) * J;

          lc.b.segment(row, nbRows) =
            lc.J.block (row, col, nbRows, Spline::NbCoeffs * rDof)
            * spline->rowParameters();
        }

        void linearize (const Splines_t& splines, const SplineOptimizationDatas_t& ss, LinearConstraint& lc)
        {
          for (std::size_t i = 0; i < functions.size(); ++i)
            linearize(splines[splineIds[i]], ss[i], i, lc);
        }

        std::vector<CollisionFunctionPtr_t> functions;
        std::vector<std::size_t> splineIds;
        std::vector<size_type> rows;
        std::vector<value_type> ratios;

        mutable Configuration_t q;
        mutable matrix_t J, Js;
        mutable typename Spline::BasisFunctionVector_t paramDerivativeCoeff;
      };

      // ----------- Resolution steps --------------------------------------- //

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Ptr_t SplineGradientBased<_PB, _SO>::create
      (const Problem& problem)
      {
	SplineGradientBased* ptr = new SplineGradientBased (problem);
	Ptr_t shPtr (ptr);
	return shPtr;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addProblemConstraints
      (const PathVectorPtr_t& init, const Splines_t& splines, LinearConstraint& lc, SplineOptimizationDatas_t& ss) const
      {
        assert (init->numberPaths() == splines.size() && ss.size() == splines.size());
        for (std::size_t i = 0; i < splines.size(); ++i) {
          addProblemConstraintOnPath (init->pathAtRank(i), i, splines[i], lc, ss[i]);
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addProblemConstraintOnPath
      (const PathPtr_t& path, const size_type& idxSpline, const SplinePtr_t& spline, LinearConstraint& lc, SplineOptimizationData& sod) const
      {
        ConstraintSetPtr_t cs = path->constraints();
        if (cs) {
          ConfigProjectorPtr_t cp = cs->configProjector();
          if (cp) {
            const HybridSolver& hs = cp->solver();
            const constraints::ExplicitSolver& es = hs.explicitSolver();

            // Get the active parameter row selection.
            value_type guessThreshold = problem().getParameter ("SplineGradientBased/guessThreshold", value_type(-1));
            Eigen::RowBlockIndices select = computeActiveParameters (path, hs, guessThreshold);

            const size_type rDof = robot_->numberDof(),
                            col  = idxSpline * Spline::NbCoeffs * rDof,
                            row = lc.J.rows(),
                            nOutVar = select.nbIndices();

            sod.set = cs;
            sod.es.reset(new ExplicitSolver(es));
            sod.activeParameters = RowBlockIndices (BlockIndex::difference
                                         (BlockIndex::segment_t(0, rDof),
                                          select.indices()));
            hppDout (info, "Path " << idxSpline << ": do not change this dof " << select);
            hppDout (info, "Path " << idxSpline << ": active dofs " << sod.activeParameters);

            // Add nOutVar constraint per coefficient.
            lc.addRows(Spline::NbCoeffs * nOutVar);
            matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
            for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
              lc.J.block  (row + k * nOutVar, col + k * rDof, nOutVar, rDof) = I;
              lc.b.segment(row + k * nOutVar, nOutVar) = I * spline->parameters().row(k).transpose();
            }

            assert ((lc.J.block(row, col, Spline::NbCoeffs * nOutVar, rDof * Spline::NbCoeffs) * spline->rowParameters())
                .isApprox(lc.b.segment(row, Spline::NbCoeffs * nOutVar)));
          }
        }
      }

      template <int _PB, int _SO>
      Eigen::RowBlockIndices SplineGradientBased<_PB, _SO>::computeActiveParameters
      (const PathPtr_t& path, const HybridSolver& hs, const value_type& guessThr, const bool& useExplicitInput) const
      {
        const constraints::ExplicitSolver& es = hs.explicitSolver();

        BlockIndex::segments_t implicitBI, explicitBI;

        // Handle implicit part
        if (hs.reducedDimension() > 0) {
          implicitBI = hs.implicitDof();

          hppDout (info, "Solver " << hs
              << '\n' << Eigen::RowBlockIndices(implicitBI));

          // in the case of PR2 passing a box from right to left hand,
          // the double grasp is a loop closure so the DoF of the base are
          // not active (one can see this in the Jacobian).
          // They should be left unconstrained.
          // TODO I do not see any good way of guessing this since it is
          // the DoF of the base are not active only on the submanifold
          // satisfying the constraint. It has to be dealt with in
          // hpp-manipulation.

          // If requested, check if the jacobian has columns of zeros.
          BlockIndex::segments_t passive;
          if (guessThr >= 0) {
            matrix_t J (hs.dimension(), es.inDers().nbIndices());
            hs.computeValue<true>(path->initial());
            hs.updateJacobian(path->initial());
            hs.getReducedJacobian(J);
            size_type j = 0, k = 0;
            for (size_type r = 0; r < J.cols(); ++r) {
              if (J.col(r).isZero(guessThr)) {
                size_type idof = es.inDers().indices()[j].first + k;
                passive.push_back(BlockIndex::segment_t (idof, 1));
                hppDout (info, "Deactivated dof (thr=" << guessThr
                    << ") " << idof << ". J = " << J.col(r).transpose());
              }
              k++;
              if (k >= es.inDers().indices()[j].second) {
                j++;
                k = 0;
              }
            }
            BlockIndex::sort(passive);
            BlockIndex::shrink(passive);
            hppDout (info, "Deactivated dof (thr=" << guessThr
                << ") " << Eigen::ColBlockIndices(passive)
                << "J = " << J);
            implicitBI = BlockIndex::difference (implicitBI, passive);
          }
        } else if (useExplicitInput) {
          Eigen::ColBlockIndices esadp = es.activeDerivativeParameters();
          implicitBI = esadp.indices();
        }

        // Handle explicit part
        explicitBI = es.outDers().indices();

        // Add both
        implicitBI.insert (implicitBI.end(),
            explicitBI.begin(), explicitBI.end());
        Eigen::RowBlockIndices rbi (implicitBI);
        rbi.updateIndices<true, true, true>();
        return rbi;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addCollisionConstraint
      (const std::size_t idxSpline,
       const SplinePtr_t& spline, const SplinePtr_t& nextSpline,
       const SplineOptimizationData& sod,
       const CollisionPathValidationReportPtr_t& report,
       LinearConstraint& collision,
       CollisionFunctions& functions) const
      {
        hppDout (info, "Collision on spline " << idxSpline << " at ratio (in [0,1]) = " << report->parameter / nextSpline->length());
        CollisionFunctionPtr_t cc =
          CollisionFunction::create (robot_, spline, nextSpline, report);

        collision.addRows(cc->outputSize());
        functions.addConstraint (cc, idxSpline,
            collision.J.rows() - 1,
            report->parameter / nextSpline->length()); 

        functions.linearize(spline, sod, functions.functions.size() - 1, collision);
      }

      template <int _PB, int _SO>
      bool SplineGradientBased<_PB, _SO>::findNewConstraint
      (LinearConstraint& constraint,
       LinearConstraint& collision,
       LinearConstraint& collisionReduced,
       CollisionFunctions& functions,
       const std::size_t iF,
       const SplinePtr_t& spline,
       const SplineOptimizationData& sod) const
      {
        HPP_SCOPE_TIMECOUNTER(SGB_findNewConstraint);
        bool solved = false;
        Configuration_t q (robot_->configSize());
        CollisionFunctionPtr_t function = functions.functions[iF];

        solved = constraint.reduceConstraint(collision, collisionReduced);

        size_type i = 5;
        while (not solved) {
          if (i == 0) {
            functions.removeLastConstraint (1, collision);
            hppDout (warning, "Could not find a suitable collision constraint. Removing it.");
            return false;
          }
          hppDout (info, "Looking for collision which does not make the constraint rank deficient.");
          // interpolate at alpha
          pinocchio::interpolate<hpp::pinocchio::LieGroupTpl>
            (robot_, function->qFree_, function->qColl_, 0.5, q);
          hppDout (info, "New q: " << q.transpose());
          // update the constraint
          function->updateConstraint (q);
          functions.linearize(spline, sod, iF, collision);
          // check the rank
          solved = constraint.reduceConstraint(collision, collisionReduced);
          --i;
        }
        return true;
      }

      // ----------- Optimize ----------------------------------------------- //

      template <int _PB, int _SO>
      PathVectorPtr_t SplineGradientBased<_PB, _SO>::optimize (const PathVectorPtr_t& path)
      {
        // Get some parameters
        value_type alphaInit = problem().getParameter ("SplineGradientBased/alphaInit", value_type(0.2));
        bool alwaysStopAtFirst = problem().getParameter ("SplineGradientBased/alwaysStopAtFirst", true);
        bool linearizeAtEachStep = problem().getParameter ("SplineGradientBased/linearizeAtEachStep", false);
        bool checkJointBound = problem().getParameter ("SplineGradientBased/checkJointBound", true);
        bool returnOptimum = problem().getParameter ("SplineGradientBased/returnOptimum", false);
        value_type costThreshold = problem().getParameter ("SplineGradientBased/costThreshold", value_type(0.01));

        PathVectorPtr_t tmp = PathVector::create (robot_->configSize(), robot_->numberDof());
        path->flatten(tmp);
        // Remove zero length path
        PathVectorPtr_t input = PathVector::create (robot_->configSize(), robot_->numberDof());
        for (std::size_t i = 0; i < tmp->numberPaths(); ++i) {
          PathPtr_t p = tmp->pathAtRank (i);
          if (p->length() > 0) input->appendPath (p);
        }

        robot_->controlComputation ((Device::Computation_t)(robot_->computationFlag() | Device::JACOBIAN));
        const size_type rDof = robot_->numberDof();

        // 1
        Splines_t splines;
        this->appendEquivalentSpline (input, splines);
        const size_type nParameters = splines.size() * Spline::NbCoeffs;

        this->initializePathValidation(splines);

        // 2
        enum { MaxContinuityOrder = int( (SplineOrder - 1) / 2) };
        const size_type orderContinuity = MaxContinuityOrder;

        LinearConstraint constraint (nParameters * rDof, 0);
        SplineOptimizationDatas_t solvers (splines.size(), SplineOptimizationData(rDof));
        addProblemConstraints (input, splines, constraint, solvers);

        this->addContinuityConstraints (splines, orderContinuity, solvers, constraint);

        // 3
        LinearConstraint collision (nParameters * rDof, 0);
        CollisionFunctions collisionFunctions;

        // 4
        // TODO add weights
        SquaredLength<Spline, 1> cost (splines, rDof, rDof);

        // 5
        bool feasible;
        { HPP_SCOPE_TIMECOUNTER(SGB_constraintDecomposition);
          feasible = constraint.decompose (true); // true = check that the constraint is feasible
        }
        if (!feasible) throw std::invalid_argument("Constraints are not feasible");

        LinearConstraint collisionReduced (constraint.PK.rows(), 0);
        constraint.reduceConstraint(collision, collisionReduced);

        LinearConstraint boundConstraint (nParameters * rDof, 0);
        if (checkJointBound) {
          this->jointBoundConstraint (splines, boundConstraint);
          if (!this->validateBounds(splines, boundConstraint).empty())
            throw std::invalid_argument("Input path does not satisfy joint bounds");
        }
        LinearConstraint boundConstraintReduced (boundConstraint.PK.rows(), 0);
        constraint.reduceConstraint(boundConstraint, boundConstraintReduced, false);

        // 6
        bool noCollision = true, stopAtFirst = alwaysStopAtFirst;
        bool minimumReached = false;

        bool computeOptimum = true, computeInterpolatedSpline = !(checkOptimum_ || returnOptimum);

        value_type alpha = (checkOptimum_ || returnOptimum ? 1 : alphaInit);

        Splines_t alphaSplines, collSplines;
        Splines_t* currentSplines;
        copy(splines, alphaSplines); copy(splines, collSplines);
        Reports_t reports;

        QuadraticProblem QP(cost.inputDerivativeSize_);
        value_type optimalCost, costLowerBound = 0;
        cost.value(optimalCost, splines);
        hppDout (info, "Initial cost is " << optimalCost);
        cost.hessian(QP.H, splines);
#ifndef NDEBUG
        checkHessian(cost, QP.H, splines);
#endif // NDEBUG

        QuadraticProblem QPc (QP, constraint);
        QPc.computeLLT();
        QPc.solve(collisionReduced, boundConstraintReduced);

        while (!(noCollision && minimumReached) && (!this->interrupt_)) {
          // 6.1
          if (computeOptimum) {
            // 6.2
            constraint.computeSolution(QPc.xStar);
            updateSplines(collSplines, constraint.xSol);
            cost.value(costLowerBound, collSplines);
            hppDout (info, "Cost interval: [" << optimalCost << ", " << costLowerBound << "]");
            currentSplines = &collSplines;
            minimumReached = true;
            computeOptimum = false;
          }
          if (computeInterpolatedSpline) {
            interpolate(splines, collSplines, alpha, alphaSplines);
            currentSplines = &alphaSplines;
            minimumReached = false;
            computeInterpolatedSpline = false;
          }

          // 6.3.2 Check for collision
          if (!returnOptimum) {
            reports = this->validatePath (*currentSplines, stopAtFirst);
            noCollision = reports.empty();
          } else {
            minimumReached = true;
            noCollision = true;
          }
          if (noCollision) {
            cost.value(optimalCost, *currentSplines);
            hppDout (info, "Cost interval: [" << optimalCost << ", " << costLowerBound << "]");
            // Update the spline
            for (std::size_t i = 0; i < splines.size(); ++i)
              splines[i]->rowParameters((*currentSplines)[i]->rowParameters());
            if (linearizeAtEachStep) {
              collisionFunctions.linearize (splines, solvers, collision);
              constraint.reduceConstraint(collision, collisionReduced);
              QPc.solve(collisionReduced, boundConstraintReduced);
              hppDout (info, "linearized");
              computeOptimum = true;
            }
            hppDout (info, "Improved path with alpha = " << alpha);

            computeInterpolatedSpline = true;
            if (!minimumReached &&
                std::abs(optimalCost - costLowerBound) < costThreshold * costLowerBound)
            {
              hppDout (info, "Stopping because cost interval is small.");
              minimumReached = true;
            }
          } else {
            if (alpha != 1.) {
              bool ok = false;
              for (std::size_t i = 0; i < reports.size(); ++i) {
                addCollisionConstraint(reports[i].second,
                    splines[reports[i].second],
                    (*currentSplines)[reports[i].second],
                    solvers[reports[i].second],
                    reports[i].first,
                    collision, collisionFunctions);

                ok |=
                  findNewConstraint (constraint, collision, collisionReduced,
                      collisionFunctions, collisionFunctions.functions.size() - 1,
                      splines[reports[i].second], solvers[reports[i].second]);
              }

              if (!ok) {
                hppDout (info, "The collision constraint would be rank deficient. Removing added constraint.");
                if (alpha < alphaInit / (1 << 2)) {
                  hppDout (info, "Interruption because alpha became too small.");
                  break;
                }
                alpha *= 0.5;
                stopAtFirst = alwaysStopAtFirst;

                computeInterpolatedSpline = true;
              } else {
                QPc.solve(collisionReduced, boundConstraintReduced);
                bool overConstrained = (QPc.H.rows() < collisionReduced.rank);
                if (overConstrained) {
                  hppDout (info, "The problem is over constrained: "
                      << QP.H.rows() << " variables for "
                      << collisionReduced.rank << " independant constraints.");
                  break;
                }
                hppDout (info, "Added " << reports.size() << " constraints. "
                    "Constraints size " << collision.J.rows() <<
                    "(rank=" << collisionReduced.rank << ", ass=" << QPc.activeSetSize << ") / " << QPc.H.cols());

                // When adding a new constraint, try first minimum under this
                // constraint. If this latter minimum is in collision,
                // re-initialize alpha to alphaInit.
                alpha = 1.;
                stopAtFirst = true;
                computeOptimum = true;
              }
            } else {
              alpha = alphaInit;
              stopAtFirst = alwaysStopAtFirst;
              computeInterpolatedSpline = true;
            }
          }
        }

        // 7
        HPP_DISPLAY_TIMECOUNTER(SGB_constraintDecomposition);
        HPP_DISPLAY_TIMECOUNTER(SGB_qpDecomposition);
        HPP_DISPLAY_TIMECOUNTER(SGB_findNewConstraint);
        HPP_DISPLAY_TIMECOUNTER(SGB_qpSolve);
        return this->buildPathVector (splines);
      }

      // ----------- Convenience functions ---------------------------------- //

      template <int _PB, int _SO>
      template <typename Cost_t>
      bool SplineGradientBased<_PB, _SO>::checkHessian
      (const Cost_t& cost, const matrix_t& H, const Splines_t& splines) const
      {
        value_type expected;
        cost.value(expected, splines);

        vector_t P (H.rows());

        const size_type size = robot_->numberDof() * Spline::NbCoeffs;
        for (std::size_t i = 0; i < splines.size(); ++i)
          P.segment (i * size, size) = splines[i]->rowParameters();
        value_type result = 0.5 * P.transpose() * H * P;

        bool ret = std::fabs(expected - result) < Eigen::NumTraits<value_type>::dummy_precision();
        if (!ret) {
          hppDout (error, "Hessian of the cost is not correct: " << expected << " - " << result << " = " << expected - result);
        }
        return ret;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::copy
      (const Splines_t& in, Splines_t& out) const
      {
        out.resize(in.size());
        for (std::size_t i = 0; i < in.size(); ++i)
          out[i] = HPP_STATIC_PTR_CAST(Spline, in[i]->copy());
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::updateSplines
      (Splines_t& splines, const vector_t& param) const
      {
        size_type row = 0, size = robot_->numberDof() * Spline::NbCoeffs;
        for (std::size_t i = 0; i < splines.size(); ++i) {
          splines[i]->rowParameters(param.segment(row, size));
          row += size;
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::interpolate
      (const Splines_t& a, const Splines_t& b, const value_type& alpha, Splines_t& res) const
      {
        assert (a.size() == b.size() && b.size() == res.size());
        assert (alpha >= 0 && alpha <= 1);

        for (std::size_t i = 0; i < a.size(); ++i)
          res[i]->rowParameters(
              (1 - alpha) * a[i]->rowParameters()
              + alpha     * b[i]->rowParameters());
      }

      // ----------- Instanciate -------------------------------------------- //

      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 1>; // equivalent to StraightPath
      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 2>;
      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 3>;
      template class SplineGradientBased<path::BernsteinBasis, 1>; // equivalent to StraightPath
      // template class SplineGradientBased<path::BernsteinBasis, 2>;
      template class SplineGradientBased<path::BernsteinBasis, 3>;
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
