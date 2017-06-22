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

#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>

#include <path-optimization/spline-gradient-based/cost.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      typedef Eigen::Map<const vector_t> ConstVectorMap_t;
      typedef Eigen::Map<      vector_t>      VectorMap_t;

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
          StraightPathPtr_t sp (HPP_DYNAMIC_PTR_CAST(StraightPath, p));
          PathVectorPtr_t pv (HPP_DYNAMIC_PTR_CAST(PathVector, p));
          if (sp)      appendEquivalentSpline (sp, splines);
          else if (pv) appendEquivalentSpline(pv, splines);
          else // if TODO check if path is a spline.
            throw std::logic_error("Unknown type of path");
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addContinuityConstraints
      (const Splines_t& splines, const size_type maxOrder, matrix_t& J, matrix_t& b)
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

        size_type row = 0,
                  // rDof = robot_->numberDof(),
                  paramSize = Spline::NbCoeffs;

        for (size_type k = 0; k <= maxOrder; ++k) {
          // TODO This assumes path lengths is 1
          // Spline::basisFunctionDerivative(k, 0, B0);
          // Spline::basisFunctionDerivative(k, 1, B1);

          // addContinuityConstraint (SplinePtr_t(), splines[0], 0, i, Jcontinuity, rhsContinuity);
          splines[0]->basisFunctionDerivative(k, 0, B0);
          bool Czero = (k == 0);
          size_type indexParam = 0;
          J.block<1, NbCoeffs> (row, indexParam) = - B0.transpose();
          if (Czero) b.row(row) = - B0.transpose() * splines[0]->parameters();
          ++row;

          for (std::size_t j = 0; j < splines.size() - 1; ++j) {
            splines[j  ]->basisFunctionDerivative(k, 1, B1);
            splines[j+1]->basisFunctionDerivative(k, 0, B0);

            // addContinuityConstraint (splines[j], splines[j+1], j, i, Jcontinuity, rhsContinuity);
            J.block<1, NbCoeffs> (row, indexParam) = B1.transpose();
            indexParam += paramSize;
            J.block<1, NbCoeffs> (row, indexParam) = - B0.transpose();

            if (Czero) b.row (row) =
                B1.transpose() * splines[j  ]->parameters()
              - B0.transpose() * splines[j+1]->parameters();
            ++row;
          }

          // addContinuityConstraint (splines.back(), SplinePtr_t(), i);
          splines.back()->basisFunctionDerivative(k, 1, B1);
          J.block<1, NbCoeffs> (row, indexParam) = B1.transpose();
          if (Czero) b.row (row) = B1.transpose() * splines.back()->parameters();
          ++row;
        }
      }

      template <typename Derived>
      void expandPK (const Eigen::MatrixBase<Derived>& PKs, matrix_t& PK, const size_type rDof)
      {
        PK.setZero();

        // matrix_t tmp = PK;
        /*
        for (size_type i = 0; i < PKs.rows(); ++i) {
          // One row is rDof constraints
          for (size_type iDof = 0; iDof < rDof; ++iDof) {
            size_type iC = i * rDof + iDof;
            for (size_type j = 0; j < PKs.cols(); ++j) {
              PK(iC, iDof + j * rDof) = PKs(i,j);
            }
          }
        }
        */
        for (size_type i = 0; i < PKs.rows(); ++i) {
          for (size_type j = 0; j < PKs.cols(); ++j) {
            PK.block(i*rDof, j*rDof, rDof, rDof).diagonal().setConstant(PKs(i,j));
          }
        }
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
        const size_type rDof = robot_->numberDof();

        // 1
        Splines_t splines;
        appendEquivalentSpline (path, splines);
        const size_type nParameters = splines.size() * Spline::NbCoeffs;

        // 2
        enum { MaxContinuityOrder = int( (SplineOrder - 1) / 2) };
        const size_type orderContinuity = MaxContinuityOrder;

        matrix_t Jcontinuity (matrix_t::Zero((splines.size() + 1) * (orderContinuity + 1), nParameters));
        matrix_t rhsContinuity (matrix_t::Zero((splines.size() + 1) * (orderContinuity + 1), rDof));

        addContinuityConstraints (splines, orderContinuity, Jcontinuity, rhsContinuity);

        isContinuous(splines, orderContinuity, Jcontinuity, rhsContinuity);

        // 3
        // addProblemConstraints

        // 4
        // TODO add weights
        SquaredLength<Spline, 1> cost (splines.size(), rDof, rDof);

        // 5
        typedef Eigen::JacobiSVD < matrix_t > SVD_t;
        SVD_t svdContinuity (Jcontinuity, Eigen::ComputeThinU | Eigen::ComputeFullV);
        // const size_type nFreeParam = svdContinuity.rank() * 
        // matrix_t Pbars (nParameters, rDof),
        assert (Jcontinuity.rows() <= Jcontinuity.cols());
        matrix_t
                 // PKs (nParameters, svdContinuity.rank()),
                 // PKs (nParameters, nParameters),
                 PK (nParameters * rDof, (Jcontinuity.cols() - svdContinuity.rank()) * rDof);

        Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Pbars
          = svdContinuity.solve (rhsContinuity);
        assert (Pbars.rows() == nParameters && Pbars.cols() == rDof);
        Eigen::Map<vector_t, Eigen::Aligned> Pbar (Pbars.data(), Pbars.size());

        // check that the constraints are feasible
        matrix_t error = Jcontinuity * Pbars - rhsContinuity;
        if (!error.isZero()) {
          hppDout (warning, "The continuity constraints are infeasible: "
              << error.norm() << '\n' << error);
        }

        // constraints::projectorOnKernel(svdContinuity, PKs);
        expandPK (constraints::getV2(svdContinuity), PK, rDof);

        // 6
        // min p^T H p   ->  min 0.5 v^t * Hr * v + br^t * v
        // s.t. J p = b
        matrix_t H (cost.inputDerivativeSize_, cost.inputDerivativeSize_);
        cost.hessian(H, splines);
#ifndef NDEBUG
        checkHessian(cost, H, splines);
#endif // NDEBUG
        matrix_t H_PK (H * PK);
        matrix_t Hr = 2 * PK.transpose() * H_PK;
        vector_t br = 2 * Pbar.transpose() * H_PK;

        // 6.2
        SVD_t svdProblem (Hr, Eigen::ComputeThinU | Eigen::ComputeThinV);
        vector_t vstar = - svdProblem.solve(br);
        vector_t Pstar = Pbar + PK * vstar;

        error = Jcontinuity * Eigen::Map<Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > (Pstar.data(), nParameters, rDof) - rhsContinuity;
        if (!error.isZero()) {
          hppDout (warning, "The continuity constraints are not satisfied: "
              << error.norm() << '\n' << error);
        }

        size_type row = 0, size = rDof * Spline::NbCoeffs;
        for (std::size_t i = 0; i < splines.size(); ++i) {
          for (size_type j = 0; j < Spline::NbCoeffs; ++j) {
            hppDout (info, "Interpolation point  " << j << " velocity: " << Pstar.segment(row + j * rDof, rDof).transpose());
            hppDout (info, "Current value is " << splines[i]->parameters().row(j));
          }
          splines[i]->rowParameters(Pstar.segment(row, size));
          row += size;
        }

        isContinuous(splines, orderContinuity, Jcontinuity, rhsContinuity);

        // 7
        return buildPathVector (splines);
      }

      // ----------- Convenience functions ---------------------------------- //

      template <int _PB, int _SO>
      bool SplineGradientBased<_PB, _SO>::isContinuous
      (const Splines_t& splines, const size_type maxOrder, const matrix_t& J, const matrix_t& b) const
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

                  // paramSize = Spline::NbCoeffs;

        matrix_t P (J.cols(), b.cols());

        size_type row = 0;
        for (std::size_t j = 0; j < splines.size(); ++j) {
          P.middleRows<NbCoeffs> (row) = splines[j]->parameters();
          row += NbCoeffs;
        }

        matrix_t error = J * P - b;

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

        bool ret = std::fabs(expected - result) < Eigen::NumTraits<value_type>::epsilon();
        if (!ret) {
          hppDout (error, "Hessian of the cost is not correct: " << expected << " - " << result << " = " << expected - result);
        }
        return ret;
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
