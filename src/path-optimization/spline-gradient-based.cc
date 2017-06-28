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

#include <hpp/util/timer.hh>

#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/collision-path-validation-report.hh>

Eigen::IOFormat IPythonFormat (Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[\n", "]\n");

#include <path-optimization/spline-gradient-based/cost.hh>
#include <path-optimization/spline-gradient-based/collision-constraint.hh>

namespace hpp {
  namespace core {
    using pinocchio::Device;

    namespace pathOptimization {
      typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix_t;
      typedef Eigen::Map<const vector_t> ConstVectorMap_t;
      typedef Eigen::Map<      vector_t>      VectorMap_t;

      HPP_DEFINE_TIMECOUNTER(SGB_validatePath);
      HPP_DEFINE_TIMECOUNTER(SGB_matrixDecomposition);

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
      struct SplineGradientBased<_PB, _SO>::LinearConstraint
      {
        LinearConstraint (size_type inputRows, size_type inputCols, size_type outputSize) :
          J (outputSize, inputRows), b (outputSize, inputCols),
          svd (outputSize, inputRows, Eigen::ComputeThinU | Eigen::ComputeFullV),
          xSol (inputRows * inputCols)
        {
          J.setZero();
          b.setZero();
        }

        template <typename Derived>
        void expandPK (const Eigen::MatrixBase<Derived>& PKs, matrix_t& PK, const size_type pSize)
        {
          PK.setZero();
          for (size_type i = 0; i < PKs.rows(); ++i) {
            for (size_type j = 0; j < PKs.cols(); ++j) {
              PK.block(i*pSize, j*pSize, pSize, pSize).diagonal().setConstant(PKs(i,j));
            }
          }
        }

        void decompose (bool check = false)
        {
          HPP_START_TIMECOUNTER(SGB_matrixDecomposition);
          if (J.rows() == 0) { // No constraint
            PK = matrix_t::Identity (J.cols() * b.cols(), J.cols() * b.cols());
            PKinv = PK;
            xStar = vector_t::Zero (PK.rows());
            return;
          }

          svd.compute (J);
          assert (J.rows() <= J.cols());

          PK.resize(J.cols() * b.cols(), (J.cols() - svd.rank()) * b.cols());
          xStar.resize (PK.rows());

          Eigen::Map<RowMajorMatrix_t, Eigen::Aligned> XStar (xStar.data(), J.cols(), b.cols());
          XStar = svd.solve (b);

          if (check) {
            // check that the constraints are feasible
            matrix_t error = J * XStar - b;
            if (!error.isZero()) {
              hppDout (warning, "The continuity constraints are infeasible: "
                  << error.norm() << '\n' << error);
            }
          }

          PKinv.resize(PK.cols(), PK.rows());
          if (b.cols() > 1) {
            expandPK (constraints::getV2(svd)          , PK   , b.cols());
            expandPK (constraints::getV2(svd).adjoint(), PKinv, b.cols());
          } else {
            PK = constraints::getV2(svd);
            PKinv = PK.adjoint();
          }
          assert((PKinv * PK).eval().isIdentity());

          HPP_STOP_AND_DISPLAY_TIMECOUNTER(SGB_matrixDecomposition);
        }

        void reduceProblem (const QuadraticProblem& QP, QuadraticProblem& QPr) const
        {
          matrix_t H_PK (QP.H * PK);
          QPr.H = PK.transpose() * H_PK;
          QPr.b = 2 * xStar.transpose() * H_PK;
          if (!QP.bIsZero) {
            QPr.b.transpose().noalias() += QP.b.transpose() * PK;
          }
          QPr.bIsZero = false;

          // QPr.Hpinv = PKinv * QP.Hpinv * PKinv.transpose();
          QPr.decompose();
        }

        bool reduceConstraint (const LinearConstraint& lc, LinearConstraint& lcr) const
        {
          lcr.J = lc.J * PK;
          lcr.b = lc.b - lc.J * xStar;

          if (lcr.J.rows() > J.cols()) {
            // The problem is over constrained
            return false;
          }

          // Decompose
          lcr.decompose(true);
          return true;
        }

        void computeSolution (const vector_t& v)
        {
          xSol = xStar + PK * v;
        }

        void addRows (const std::size_t& nbRows)
        {
          J.conservativeResize(J.rows() + nbRows, J.cols());
          b.conservativeResize(b.rows() + nbRows, b.cols());

          J.bottomRows(nbRows).setZero();
        }

        // model
        matrix_t J;
        matrix_t b;

        // Data
        Eigen::JacobiSVD < matrix_t > svd;

        // Data for vectorized input
        // Solutions are x = xStar + PK * v, v \in kernel(J)
        matrix_t PK, PKinv;
        vector_t xStar, xSol;
      };

      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::QuadraticProblem
      {
        QuadraticProblem (size_type inputSize) :
          H (inputSize, inputSize), b (inputSize),
          Hpinv (inputSize, inputSize),
          svd (inputSize, inputSize, Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (inputSize)
        {
          H.setZero();
          b.setZero();
          bIsZero = true;
        }

        QuadraticProblem (const QuadraticProblem& QP, const LinearConstraint& lc) :
          H (lc.PK.cols(), lc.PK.cols()), b (lc.PK.cols()),
          Hpinv (lc.PK.cols(), lc.PK.cols()),
          svd (lc.PK.cols(), lc.PK.cols(), Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (lc.PK.cols())
        {
          lc.reduceProblem(QP, *this);
        }

        QuadraticProblem (const QuadraticProblem& QP) :
          H (QP.H), b (QP.b), Hpinv (QP.Hpinv), svd (QP.svd), xStar (QP.xStar)
        {}

        void addRows (const std::size_t& nbRows)
        {
          H.conservativeResize(H.rows() + nbRows, H.cols());
          b.conservativeResize(b.rows() + nbRows, b.cols());

          H.bottomRows(nbRows).setZero();
        }

        /// \param blockSize if not zero, H is considered block diagonal, the
        ///                  block being of size blockSize x blockSize
        bool decompose (std::size_t blockSize = 0)
        {
          HPP_START_TIMECOUNTER(SGB_matrixDecomposition);
          bool isFullRank = true;
          if (blockSize == 0) {
            svd.compute(H);
            constraints::pseudoInverse (svd, Hpinv);
            isFullRank = (svd.rank() == H.rows());
          } else {
            Hpinv.setZero();
            Eigen::JacobiSVD<matrix_t> svdTmp (blockSize, blockSize, Eigen::ComputeThinU | Eigen::ComputeThinV);
            for (size_type r = 0; r < H.rows(); r += blockSize) {
#ifndef NDEBUG
              for (size_type c = 0; c < H.rows(); c += blockSize) {
                if (c == r) continue;
                if (!H.block(r, c, blockSize, blockSize).isZero()) {
                  hppDout (error, "Hessian should be block diagonal (blockSize = " << blockSize << ").\n"
                      << H.block(r, c, blockSize, blockSize));
                }
              }
#endif // NDEBUG
              svdTmp.compute(H.block(r, r, blockSize, blockSize));
              constraints::pseudoInverse (svdTmp, Hpinv.block(r, r, blockSize, blockSize));
              isFullRank = isFullRank && (svdTmp.rank() == (size_type)blockSize);
            }
          }
          HPP_STOP_AND_DISPLAY_TIMECOUNTER(SGB_matrixDecomposition);
          return isFullRank;
        }

        void solve ()
        {
          xStar = - 0.5 * Hpinv * b;
        }

        // model
        matrix_t H;
        vector_t b;
        bool bIsZero;

        // Data
        matrix_t Hpinv;
        Eigen::JacobiSVD<matrix_t> svd;
        vector_t xStar;
      };

      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::CollisionFunctions
      {
        void addConstraint (const CollisionFunctionPtr_t& f,
                            const std::size_t& idx,
                            const value_type& r)
        {
          assert (f->outputSize() == 1);
          functions.push_back(f);
          indexes.push_back(idx);
          ratios.push_back(r);
        }

        // Compute linearization
        // b = f(S(t))
        // J = Jf(S(p, t)) * dS/dp
        // f(S(t)) = b -> J * P = b
        void linearize (const SplinePtr_t& spline, const std::size_t& fIdx,
            LinearConstraint& lc)
        {
          const CollisionFunctionPtr_t& f = functions[fIdx];

          const size_type row = fIdx,
                          nbRows = 1,
                          rDof = f->inputDerivativeSize();
          const value_type t = spline->length() * ratios[fIdx];

          q.resize(f->inputSize());
          (*spline) (q, t);

          f->value(lc.b.col(0).segment(row, nbRows), q);

          J.resize(f->outputSize(), f->inputDerivativeSize());
          f->jacobian(J, q);

          spline->parameterDerivativeCoefficients(paramDerivativeCoeff, t);

          const size_type col = indexes[fIdx] * Spline::NbCoeffs * rDof;
          for (size_type i = 0; i < Spline::NbCoeffs; ++i)
            lc.J.block (row, col + i * rDof, nbRows, rDof).noalias()
              = paramDerivativeCoeff(i) * J;
        }

        void linearize (const Splines_t& splines, LinearConstraint& lc)
        {
          for (std::size_t i = 0; i < functions.size(); ++i)
            linearize(splines[indexes[i]], i, lc);
        }

        std::vector<CollisionFunctionPtr_t> functions;
        std::vector<std::size_t> indexes;
        std::vector<value_type> ratios;

        mutable Configuration_t q;
        mutable matrix_t J;
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
        splines.push_back(
            HPP_DYNAMIC_PTR_CAST(Spline,
            steeringMethod_->impl_compute (path->initial(), path->end()))
            );
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
          SplinePtr_t spline         (HPP_DYNAMIC_PTR_CAST(Spline, p));
          if (straight   ) appendEquivalentSpline(straight, splines);
          else if (pvect ) appendEquivalentSpline(pvect   , splines);
          else if (spline) splines.push_back(HPP_STATIC_PTR_CAST(Spline, spline->copy()));
          else // if TODO check if path is another type of spline.
            throw std::logic_error("Unknown type of path");
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addContinuityConstraints
      (const Splines_t& splines, const size_type maxOrder, LinearConstraint& lc)
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

        size_type row = 0, paramSize = Spline::NbCoeffs;

        for (size_type k = 0; k <= maxOrder; ++k) {

          // Continuity at the beginning
          splines[0]->basisFunctionDerivative(k, 0, B0);
          bool Czero = (k == 0);
          size_type indexParam = 0;
          lc.J.template block<1, NbCoeffs> (row, indexParam) = - B0.transpose();
          if (Czero) lc.b.row(row) = - B0.transpose() * splines[0]->parameters();
          ++row;

          for (std::size_t j = 0; j < splines.size() - 1; ++j) {
            splines[j  ]->basisFunctionDerivative(k, 1, B1);
            splines[j+1]->basisFunctionDerivative(k, 0, B0);

            // Continuity between spline i and j
            lc.J.template block<1, NbCoeffs> (row, indexParam) = B1.transpose();
            indexParam += paramSize;
            lc.J.template block<1, NbCoeffs> (row, indexParam) = - B0.transpose();

            if (Czero) lc.b.row (row) =
                B1.transpose() * splines[j  ]->parameters()
              - B0.transpose() * splines[j+1]->parameters();
            ++row;
          }

          // Continuity at the end
          splines.back()->basisFunctionDerivative(k, 1, B1);
          lc.J.template block<1, NbCoeffs> (row, indexParam) = B1.transpose();
          if (Czero) lc.b.row (row) = B1.transpose() * splines.back()->parameters();
          ++row;
        }
      }

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Reports_t SplineGradientBased<_PB, _SO>::validatePath
      (const Splines_t& splines, bool stopAtFirst) const
      {
        HPP_START_TIMECOUNTER(SGB_validatePath);
        PathValidationPtr_t pathValidation (problem ().pathValidation ());
	PathPtr_t validPart;
	PathValidationReportPtr_t report;
	Reports_t reports;
        for (std::size_t i = 0; i < splines.size(); ++i) {
	  if (!pathValidation->validate (splines[i], false, validPart, report)) {
	    HPP_STATIC_CAST_REF_CHECK (CollisionPathValidationReport, *report);
	    reports.push_back
	      (std::make_pair (HPP_STATIC_PTR_CAST
			       (CollisionPathValidationReport, report), i));
            if (stopAtFirst) break;
	  }
	}
        HPP_STOP_AND_DISPLAY_TIMECOUNTER(SGB_validatePath);
        return reports;
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addCollisionConstraint
      (const std::size_t idxSpline,
       const SplinePtr_t& spline, const SplinePtr_t& nextSpline,
       const CollisionPathValidationReportPtr_t& report,
       LinearConstraint& collision,
       CollisionFunctions& functions) const
      {
        CollisionFunctionPtr_t cc =
          CollisionFunction::create (robot_, spline, nextSpline, report);

        collision.addRows(cc->outputSize());
        functions.addConstraint (cc, idxSpline, report->parameter / nextSpline->length()); 

        functions.linearize(spline, functions.functions.size() - 1, collision);
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
        bool alwaysStopAtFirst = problem().getParameter ("SplineGradientBased/alwaysStopAtFirst", false);
        bool linearizeAtEachStep = problem().getParameter ("SplineGradientBased/linearizeAtEachStep", true);

        robot_->controlComputation ((Device::Computation_t)(robot_->computationFlag() | Device::JACOBIAN));
        const size_type rDof = robot_->numberDof();

        // 1
        Splines_t splines;
        appendEquivalentSpline (path, splines);
        const size_type nParameters = splines.size() * Spline::NbCoeffs;

        // 2
        enum { MaxContinuityOrder = int( (SplineOrder - 1) / 2) };
        const size_type orderContinuity = MaxContinuityOrder;

        LinearConstraint continuity (nParameters, rDof, (splines.size() + 1) * (orderContinuity + 1));
        addContinuityConstraints (splines, orderContinuity, continuity);
        isContinuous(splines, orderContinuity, continuity);

        // 3
        // addProblemConstraints
        LinearConstraint collision (nParameters * rDof, 1, 0);
        CollisionFunctions collisionFunctions;

        // 4
        // TODO add weights
        SquaredLength<Spline, 1> cost (splines, rDof, rDof);

        // 5
        continuity.decompose (true); // true = check that the constraint is feasible
        LinearConstraint collisionReduced (continuity.PK.rows(), 1, 0);
        continuity.reduceConstraint(collision, collisionReduced);

        // 6
        bool noCollision = true, stopAtFirst = true;
        bool minimumReached = false;
        value_type alpha = 1;
        Splines_t alphaSplines, collSplines;
        Splines_t* currentSplines;
        copy(splines, alphaSplines); copy(splines, collSplines);
        Reports_t reports;

        QuadraticProblem QP(cost.inputDerivativeSize_);
        cost.hessian(QP.H, splines);
#ifndef NDEBUG
        checkHessian(cost, QP.H, splines);
#endif // NDEBUG
        QP.decompose(Spline::NbCoeffs * rDof); // Use the fact that the Hessian is block diagonal.

        QuadraticProblem QPcontinuous (QP, continuity);
        // TODO avoid this copy
        QuadraticProblem QPreduced (QPcontinuous);

        while (!(noCollision && minimumReached) && (!interrupt_)) {
          // 6.1
          // min p^T H p   ->  min 0.5 v^t * Hr * v + br^t * v
          // s.t. J p = b
          if (alpha == 1) {
            // 6.2
            QPreduced.solve();
            collisionReduced.computeSolution(QPreduced.xStar);
            continuity.computeSolution(collisionReduced.xSol);
            updateSplines(collSplines, continuity.xSol);
            currentSplines = &collSplines;
            minimumReached = true;
          } else {
            interpolate(splines, collSplines, alpha, alphaSplines);
            currentSplines = &alphaSplines;
            minimumReached = false;
          }

          // 6.3
          reports = validatePath (*currentSplines, stopAtFirst);
          noCollision = reports.empty();
          if (noCollision) {
            // Update the spline
            for (std::size_t i = 0; i < splines.size(); ++i)
              splines[i]->rowParameters((*currentSplines)[i]->rowParameters());
            if (linearizeAtEachStep) {
              collisionFunctions.linearize (splines, collision);
              continuity.reduceConstraint(collision, collisionReduced);
              collisionReduced.reduceProblem (QPcontinuous, QPreduced);
            }
            hppDout (info, "Improved path with alpha = " << alpha);
          } else {
            if (alpha != 1.) {
              hppDout (info, "Adding " << reports.size() << " constraints.");
              for (std::size_t i = 0; i < reports.size(); ++i)
                addCollisionConstraint(reports[i].second,
                    splines[reports[i].second],
                    (*currentSplines)[reports[i].second],
                    reports[i].first,
                    collision, collisionFunctions);

              bool feasible = continuity.reduceConstraint(collision, collisionReduced);
              if (!feasible) {
                hppDout (info, "The constraints became infeasible.");
                break;
              }
              QPreduced = QuadraticProblem (QPcontinuous, collisionReduced);

              // When adding a new constraint, try first minimum under this
              // constraint. If this latter minimum is in collision,
              // re-initialize alpha to alphaInit.
              alpha = 1.;
              stopAtFirst = true;
            } else {
              alpha = alphaInit;
              stopAtFirst = alwaysStopAtFirst;
            }
          }

          isContinuous(splines, orderContinuity, continuity);
        }

        // 7
        return buildPathVector (splines);
      }

      // ----------- Convenience functions ---------------------------------- //

      template <int _PB, int _SO>
      bool SplineGradientBased<_PB, _SO>::isContinuous
      (const Splines_t& splines, const size_type maxOrder, const LinearConstraint& lc) const
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
