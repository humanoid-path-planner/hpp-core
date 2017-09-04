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
#include <hpp/core/path-optimization/spline-gradient-based/linear-constraint.hh>

#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>

#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/collision-path-validation-report.hh>

Eigen::IOFormat IPythonFormat (Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[\n", "]\n");

#include <path-optimization/spline-gradient-based/cost.hh>
#include <path-optimization/spline-gradient-based/collision-constraint.hh>
#include <path-optimization/spline-gradient-based/joint-bounds.hh>
#include <path-optimization/spline-gradient-based/eiquadprog_2011.hpp>

namespace hpp {
  namespace core {
    using pinocchio::Device;

    namespace pathOptimization {
      typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix_t;
      typedef Eigen::Map<const vector_t> ConstVectorMap_t;
      typedef Eigen::Map<      vector_t>      VectorMap_t;

      typedef Eigen::BlockIndex<size_type> BlockIndex;

      HPP_DEFINE_TIMECOUNTER(SGB_validatePath);
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
        : PathOptimizer (problem),
        robot_ (problem.robot()),
        steeringMethod_(SM_t::create(problem))
      {}

      // ----------- Convenience class -------------------------------------- //

      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::ContinuityConstraint
      {
        ContinuityConstraint (size_type inputRows, size_type inputCols, size_type outputSize) :
          J (outputSize, inputRows), b (outputSize, inputCols)
        {
          J.setZero();
          b.setZero();
        }

        template <typename Derived>
        static void expand (const Eigen::MatrixBase<Derived>& in, matrix_t& out, const size_type pSize)
        {
          out.setZero();
          for (size_type i = 0; i < in.rows(); ++i) {
            for (size_type j = 0; j < in.cols(); ++j) {
              out.block(i*pSize, j*pSize, pSize, pSize).diagonal().setConstant(in(i,j));
            }
          }
        }

        LinearConstraint linearConstraint () const
        {
          LinearConstraint lc (J.cols() * b.cols(), b.size());
          expand (J, lc.J, b.cols());
          Eigen::Map<RowMajorMatrix_t> (lc.b.data(), b.rows(), b.cols()) = b;
          return lc;
        }

        typedef typename Spline::BasisFunctionVector_t BasisFunctionVector_t;

        static inline void setRows (LinearConstraint& lc, const size_type& row,
            const size_type& col,
            const size_type& rDof,
            const RowBlockIndexes select,
            const BasisFunctionVector_t& Bl,
            const BasisFunctionVector_t& Br,
            const SplinePtr_t& splineL,
            const SplinePtr_t& splineR,
            bool Czero)
        {
          const size_type& rows = select.nbIndexes();
          size_type c = col;
          lc.b.segment(row, rows).setZero();
          if (splineL) {
            for (size_type i = 0; i < Spline::NbCoeffs; ++i) {
              lc.J.block (row, c, rows, rDof).noalias()
                = select.rview ( - Bl(i) * matrix_t::Identity(rDof, rDof));
              c += rDof;
            }
            if (Czero)
              lc.b.segment(row, rows).noalias()
                = select.rview (splineL->parameters ().transpose() * (-Bl));
          }
          if (splineR) {
            for (size_type i = 0; i < Spline::NbCoeffs; ++i) {
              lc.J.block (row, c, rows, rDof).noalias()
                = select.rview ( Br(i) * matrix_t::Identity(rDof, rDof));
              c += rDof;
            }
            if (Czero)
              lc.b.segment(row, rows).noalias()
                += select.rview (splineR->parameters ().transpose() * Br).eval();
          }
        }

        // model
        matrix_t J;
        matrix_t b;
      };

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
          lc.reduceProblem(QP, *this);
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
        void linearize (const SplinePtr_t& spline, const SolverPtr_t& solver,
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
          if (solver.es) solver.es->solve(q);

          vector_t v (f->outputSize());
          f->value(v, q);

          J.resize(f->outputSize(), f->inputDerivativeSize());
          f->jacobian(J, q);

          // Apply chain rule if necessary
          if (solver.es) {
            Js.resize(solver.es->derSize(), solver.es->derSize());
            solver.es->jacobian(Js, q);

            solver.es->inDers().lview(J) =
              solver.es->inDers().lview(J).eval() +
              solver.es->outDers().rviewTranspose(J).eval()
              * solver.es->viewJacobian(Js).eval(); 
            solver.es->outDers().lviewTranspose(J).setZero();
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

        void linearize (const Splines_t& splines, const Solvers_t& ss, LinearConstraint& lc)
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
      void SplineGradientBased<_PB, _SO>::appendEquivalentSpline
      (const StraightPathPtr_t& path, Splines_t& splines) const
      {
        PathPtr_t s =
          steeringMethod_->impl_compute (path->initial(), path->end());
        if (path->constraints()) {
          splines.push_back(
              HPP_DYNAMIC_PTR_CAST(Spline, s->copy (path->constraints()))
              );
        } else {
          splines.push_back(HPP_DYNAMIC_PTR_CAST(Spline, s));
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::appendEquivalentSpline
      (const InterpolatedPathPtr_t& path, Splines_t& splines) const
      {
        if (path->interpolationPoints().size() > 2) {
          HPP_THROW (std::logic_error,
              "Projected path with more than 2 IPs are not supported. "
              << path->interpolationPoints().size() << ").");
        }
        PathPtr_t s =
          steeringMethod_->impl_compute (path->initial(), path->end());
        if (path->constraints()) {
          splines.push_back(
              HPP_DYNAMIC_PTR_CAST(Spline, s->copy (path->constraints()))
              );
        } else {
          splines.push_back(HPP_DYNAMIC_PTR_CAST(Spline, s));
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::appendEquivalentSpline
      (const PathVectorPtr_t& path, Splines_t& splines) const
      {
        for (std::size_t i = 0; i < path->numberPaths(); ++i)
        {
          PathPtr_t p = path->pathAtRank(i);
          StraightPathPtr_t straight (HPP_DYNAMIC_PTR_CAST(StraightPath, p));
          PathVectorPtr_t pvect      (HPP_DYNAMIC_PTR_CAST(PathVector, p));
          InterpolatedPathPtr_t intp (HPP_DYNAMIC_PTR_CAST(InterpolatedPath, p));
          SplinePtr_t spline         (HPP_DYNAMIC_PTR_CAST(Spline, p));
          if (straight   ) appendEquivalentSpline(straight, splines);
          else if (pvect ) appendEquivalentSpline(pvect   , splines);
          else if (intp  ) appendEquivalentSpline(intp    , splines);
          else if (spline) splines.push_back(HPP_STATIC_PTR_CAST(Spline, spline->copy()));
          else // if TODO check if path is another type of spline.
            throw std::logic_error("Unknown type of path");
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::initializePathValidation
      (const Splines_t& splines)
      {
        validations_.resize(splines.size());
        for (std::size_t i = 0; i < splines.size(); ++i) {
          validations_[i] = problem ().pathValidation();
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addProblemConstraints
      (const PathVectorPtr_t& init, const Splines_t& splines, LinearConstraint& lc, Solvers_t& ss) const
      {
        assert (init->numberPaths() == splines.size() && ss.size() == splines.size());
        for (std::size_t i = 0; i < splines.size(); ++i) {
          addProblemConstraintOnPath (init->pathAtRank(i), i, splines[i], lc, ss[i]);
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addProblemConstraintOnPath
      (const PathPtr_t& path, const size_type& idxSpline, const SplinePtr_t& spline, LinearConstraint& lc, SolverPtr_t& solver) const
      {
        ConstraintSetPtr_t cs = path->constraints();
        if (cs) {
          ConfigProjectorPtr_t cp = cs->configProjector();
          if (cp) {
            const HybridSolver& hs = cp->solver();
            const constraints::ExplicitSolver& es = hs.explicitSolver();

            // Get the active parameter row selection.
            value_type guessThreshold = problem().getParameter ("SplineGradientBased/guessThreshold", value_type(-1));
            Eigen::RowBlockIndexes select = computeActiveParameters (path, hs, guessThreshold);

            const size_type rDof = robot_->numberDof(),
                            col  = idxSpline * Spline::NbCoeffs * rDof,
                            row = lc.J.rows(),
                            nOutVar = select.nbIndexes();

            solver.set = cs;
            solver.es.reset(new Solver_t(es));
            solver.av = RowBlockIndexes (BlockIndex::difference (BlockIndex::type(0, rDof), select.indexes()));
            hppDout (info, "Path " << idxSpline << ": do not change this dof " << select);
            hppDout (info, "Path " << idxSpline << ": active dofs " << solver.av);

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
      Eigen::RowBlockIndexes SplineGradientBased<_PB, _SO>::computeActiveParameters
      (const PathPtr_t& path, const HybridSolver& hs, const value_type& guessThr, const bool& useExplicitInput) const
      {
        const constraints::ExplicitSolver& es = hs.explicitSolver();

        BlockIndex::vector_t implicitBI, explicitBI;

        // Handle implicit part
        if (hs.dimension() > 0) {
          // This set of active parameters does not take the explicit
          // function into accounts.
          constraints::bool_array_t adp = hs.activeDerivativeParameters();
          implicitBI = BlockIndex::fromLogicalExpression(adp);

          // Add active parameters from explicit solver.
          Eigen::ColBlockIndexes esadp = es.activeDerivativeParameters();
          implicitBI.insert (implicitBI.end(),
              esadp.indexes().begin(), esadp.indexes().end());
          BlockIndex::sort(implicitBI);
          BlockIndex::shrink(implicitBI);

          // in the case of PR2 passing a box from right to left hand,
          // the double grasp is a loop closure so the DoF of the base are
          // not active (one can see this in the Jacobian).
          // They should be left unconstrained.
          // TODO I do not see any good way of guessing this since it is
          // the DoF of the base are not active only on the submanifold
          // satisfying the constraint. It has to be dealt with in
          // hpp-manipulation.

          // If requested, check if the jacobian has columns of zeros.
          BlockIndex::vector_t passive;
          if (guessThr >= 0) {
            matrix_t J (hs.dimension(), es.inDers().nbIndexes());
            hs.computeValue<true>(path->initial());
            hs.updateJacobian(path->initial());
            hs.getReducedJacobian(J);
            size_type j = 0, k = 0;
            for (size_type r = 0; r < J.cols(); ++r) {
              if (J.col(r).isZero(guessThr)) {
                size_type idof = es.inDers().indexes()[j].first + k;
                passive.push_back(BlockIndex::type(idof, 1));
                hppDout (info, "Deactivated dof (thr=" << guessThr
                    << ") " << idof << ". J = " << J.col(r).transpose());
              }
              k++;
              if (k >= es.inDers().indexes()[j].second) {
                j++;
                k = 0;
              }
            }
            BlockIndex::sort(passive);
            BlockIndex::shrink(passive);
            hppDout (info, "Deactivated dof (thr=" << guessThr
                << ") " << Eigen::ColBlockIndexes(passive)
                << "J = " << J);
            implicitBI = BlockIndex::difference (implicitBI, passive);
          }
        } else if (useExplicitInput) {
          Eigen::ColBlockIndexes esadp = es.activeDerivativeParameters();
          implicitBI = esadp.indexes();
        }

        // Handle explicit part
        explicitBI = es.outDers().indexes();

        // Add both
        implicitBI.insert (implicitBI.end(),
            explicitBI.begin(), explicitBI.end());
        Eigen::RowBlockIndexes rbi (implicitBI);
        rbi.updateIndexes<true, true, true>();
        return rbi;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addContinuityConstraints
      (const Splines_t& splines, const size_type maxOrder,
       const Solvers_t& ss, LinearConstraint& lc)
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

        const size_type rDof = robot_->numberDof();

        // Compute which continuity constraint are necessary
        size_type nbRows = 0;
        std::vector<RowBlockIndexes> rbis (splines.size() + 1);
        BlockIndex::type space (0, rDof);
        rbis[0] = ss[0].av;
        nbRows += rbis[0].nbIndexes();
        for (std::size_t i = 1; i < ss.size(); ++i) {
          // Compute union between A = ss[i-1].av.indexes()
          // and B = ss[i].av.indexes()
          BlockIndex::vector_t v (ss[i-1].av.indexes());
          v.insert(v.end(), ss[i].av.indexes().begin(), ss[i].av.indexes().end());
          rbis[i] = RowBlockIndexes (v);
          rbis[i].updateRows<true, true, true>();
          hppDout (info, "Optimize waypoint " << i << " over " << rbis[i]);
          nbRows += rbis[i].nbIndexes();
        }
        rbis[ss.size()] = ss[ss.size()-1].av;
        nbRows += rbis[ss.size()].nbIndexes();

        nbRows *= (maxOrder + 1);

        // Create continuity constraint
        size_type row = lc.J.rows(), paramSize = rDof * Spline::NbCoeffs;
        lc.addRows (nbRows);

        for (size_type k = 0; k <= maxOrder; ++k) {
          bool Czero = (k == 0);

          // Continuity at the beginning
          splines[0]->basisFunctionDerivative(k, 0, B0);
          size_type indexParam = 0;

          ContinuityConstraint::setRows
            (lc, row, indexParam, rDof, rbis[0],
             B1, B0, SplinePtr_t(), splines[0], Czero);
          row += rbis[0].nbIndexes();

          for (std::size_t j = 0; j < splines.size() - 1; ++j) {
            splines[j  ]->basisFunctionDerivative(k, 1, B1);
            splines[j+1]->basisFunctionDerivative(k, 0, B0);

            // Continuity between spline i and j
            ContinuityConstraint::setRows
              (lc, row, indexParam, rDof, rbis[j+1],
               B1, B0, splines[j], splines[j+1], Czero);

            row += rbis[j+1].nbIndexes();
            indexParam += paramSize;
          }

          // Continuity at the end
          splines.back()->basisFunctionDerivative(k, 1, B1);
          ContinuityConstraint::setRows
            (lc, row, indexParam, rDof, rbis.back(),
             B1, B0, splines.back(), SplinePtr_t(), Czero);
          row += rbis.back().nbIndexes();

          assert (indexParam + paramSize == lc.J.cols());
        }
        assert(row == lc.J.rows());
      }

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Reports_t SplineGradientBased<_PB, _SO>::validatePath
      (const Splines_t& splines, bool stopAtFirst) const
      {
        assert (validations_.size() == splines.size());
        HPP_SCOPE_TIMECOUNTER(SGB_validatePath);
	PathPtr_t validPart;
	PathValidationReportPtr_t report;
	Reports_t reports;
        for (std::size_t i = 0; i < splines.size(); ++i) {
	  if (!validations_[i]->validate (splines[i], false, validPart, report)) {
	    HPP_STATIC_CAST_REF_CHECK (CollisionPathValidationReport, *report);
	    reports.push_back
	      (std::make_pair (HPP_STATIC_PTR_CAST
			       (CollisionPathValidationReport, report), i));
            if (stopAtFirst) break;
	  }
	}
        return reports;
      }

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Indexes_t SplineGradientBased<_PB, _SO>::validateBounds
      (const Splines_t& splines, const LinearConstraint& lc) const
      {
        Indexes_t violated;

        const size_type rDof = robot_->numberDof();
        const size_type cols = rDof * Spline::NbCoeffs;
        const size_type rows = lc.J.rows() / splines.size();
        const size_type nBoundedDof = rows / (2*Spline::NbCoeffs);
        assert (lc.J.rows() % splines.size() == 0);
        assert (rows % (2*Spline::NbCoeffs) == 0);

        const value_type thr = Eigen::NumTraits<value_type>::dummy_precision();
        vector_t err;
        std::vector<bool> dofActive (nBoundedDof, false);

        for (std::size_t i = 0; i < splines.size(); ++i) {
          const size_type row = i * rows;
          const size_type col = i * cols;

          err = lc.J.block(row, col, rows, cols) * splines[i]->rowParameters()
            - lc.b.segment(row, rows);

          // Find the maximum per parameter
          for (size_type j = 0; j < nBoundedDof; ++j) {
            if (dofActive[j]) continue;
            value_type errM = -thr;
            size_type iErrM = -1;
            for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
              const size_type low = 2*j + k * 2*nBoundedDof;
              const size_type up  = 2*j + k * 2*nBoundedDof + 1;
              if (err(low) < errM) {
                iErrM = low;
                errM = err(low);
              }
              if (err(up) < errM) {
                iErrM = up;
                errM = err(up);
              }
            }
            if (iErrM >= 0) {
              violated.push_back(row + iErrM);
              dofActive[j] = true;
              hppDout(info, "Bound violation at spline " << i << ", param " << (iErrM - 2*j) / nBoundedDof
                  << ", iDof " << j << ": " << errM);
            }
          }
	}
        if (!violated.empty()) {
          hppDout (info, violated.size() << " bounds violated." );
        }
        return violated;
      }

      template <int _PB, int _SO>
      std::size_t SplineGradientBased<_PB, _SO>::addBoundConstraints
      (const Indexes_t& bci, const LinearConstraint& bc,
       Bools_t& activeConstraint, LinearConstraint& constraint) const
      {
        std::size_t nbC = 0;
        for (std::size_t i = 0; i < bci.size(); i++) {
          if (!activeConstraint[bci[i]]) {
            ++nbC;
            constraint.addRows (1);
            constraint.J.template bottomRows<1>() = bc.J.row(bci[i]);
            constraint.b(constraint.b.size() - 1) = bc.b (bci[i]);
          }
        }
        return nbC;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addCollisionConstraint
      (const std::size_t idxSpline,
       const SplinePtr_t& spline, const SplinePtr_t& nextSpline,
       const SolverPtr_t& solver,
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

        functions.linearize(spline, solver, functions.functions.size() - 1, collision);
      }

      template <int _PB, int _SO>
      bool SplineGradientBased<_PB, _SO>::findNewConstraint
      (LinearConstraint& constraint,
       LinearConstraint& collision,
       LinearConstraint& collisionReduced,
       CollisionFunctions& functions,
       const std::size_t iF,
       const SplinePtr_t& spline,
       const SolverPtr_t& solver) const
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
          functions.linearize(spline, solver, iF, collision);
          // check the rank
          solved = constraint.reduceConstraint(collision, collisionReduced);
          --i;
        }
        return true;
      }

      template <int _PB, int _SO>
      PathVectorPtr_t SplineGradientBased<_PB, _SO>::buildPathVector
      (const Splines_t& splines) const
      {
        PathVectorPtr_t pv =
          PathVector::create (robot_->configSize(), robot_->numberDof());

        for (std::size_t i = 0; i < splines.size(); ++i)
          pv->appendPath (splines[i]);
        return pv;
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
        appendEquivalentSpline (input, splines);
        const size_type nParameters = splines.size() * Spline::NbCoeffs;

        initializePathValidation(splines);

        // 2
        enum { MaxContinuityOrder = int( (SplineOrder - 1) / 2) };
        const size_type orderContinuity = MaxContinuityOrder;

        LinearConstraint constraint (nParameters * rDof, 0);
        Solvers_t solvers (splines.size(), SolverPtr_t(rDof));
        addProblemConstraints (input, splines, constraint, solvers);

        addContinuityConstraints (splines, orderContinuity, solvers, constraint);

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
          jointBoundConstraint (splines, boundConstraint);
          if (!validateBounds(splines, boundConstraint).empty())
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

        while (!(noCollision && minimumReached) && (!interrupt_)) {
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
            reports = validatePath (*currentSplines, stopAtFirst);
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
        HPP_DISPLAY_TIMECOUNTER(SGB_validatePath);
        HPP_DISPLAY_TIMECOUNTER(SGB_constraintDecomposition);
        HPP_DISPLAY_TIMECOUNTER(SGB_qpDecomposition);
        HPP_DISPLAY_TIMECOUNTER(SGB_findNewConstraint);
        HPP_DISPLAY_TIMECOUNTER(SGB_qpSolve);
        return buildPathVector (splines);
      }

      // ----------- Convenience functions ---------------------------------- //

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::jointBoundConstraint
      (const Splines_t& splines, LinearConstraint& lc) const
      {
        const size_type rDof = robot_->numberDof();
        const size_type cols = Spline::NbCoeffs * rDof;

        matrix_t A (2*rDof, rDof);
        vector_t b (2*rDof);
        const size_type rows = jointBoundMatrices (robot_, robot_->neutralConfiguration(), A, b);

        A.resize(rows, rDof);
        b.resize(rows);

        const size_type size = Spline::NbCoeffs * rows;
        lc.J.resize(splines.size() * size, splines.size() * cols);
        lc.b.resize(splines.size() * size);

        lc.J.setZero();
        lc.b.setZero();

        for (std::size_t i = 0; i < splines.size(); ++i) {
          jointBoundMatrices (robot_, splines[i]->base(), A, b);
          for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
            const size_type row = i * size + k * rows;
            const size_type col = i * cols + k * rDof;
            lc.J.block (row, col, rows, rDof) = -A;
            lc.b.segment (row, rows)          = -b;
          }
        }
      }

      template <int _PB, int _SO>
      bool SplineGradientBased<_PB, _SO>::isContinuous
      (const Splines_t& splines, const size_type maxOrder, const ContinuityConstraint& lc) const
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

        matrix_t P (lc.J.cols(), lc.b.cols());

        size_type row = 0;
        for (std::size_t j = 0; j < splines.size(); ++j) {
          P.middleRows<NbCoeffs> (row) = splines[j]->parameters();
          row += NbCoeffs;
        }

        matrix_t error = lc.J * P - lc.b;

        bool result = true;
        row = 0;
        for (size_type k = 0; k <= maxOrder; ++k) {
          for (size_type j = -1; j < (size_type)splines.size(); ++j) {
            if (!error.row (row).isZero ()) {
              hppDout (error, "Continuity constraint " << row << " not satisfied. Order = " << k);
              if (j < 0) {
                hppDout (error, "Start position");
              } else if (j == (size_type)(splines.size() - 1)) {
                hppDout (error, "End position");
              } else {
                hppDout (error, "Between path " << j << " and " << j+1);
              }
              hppDout(error, "Error is " << error.row(row));
              result = false;
            }
            ++row;
          }
        }

        return result;
      }

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

      template class SplineGradientBased<path::CanonicalPolynomeBasis, 1>; // equivalent to StraightPath
      template class SplineGradientBased<path::CanonicalPolynomeBasis, 2>;
      template class SplineGradientBased<path::CanonicalPolynomeBasis, 3>;
      template class SplineGradientBased<path::BernsteinBasis, 1>; // equivalent to StraightPath
      template class SplineGradientBased<path::BernsteinBasis, 2>;
      template class SplineGradientBased<path::BernsteinBasis, 3>;
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
