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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH

#include <hpp/util/debug.hh>

#include <hpp/core/path/spline.hh>
#include <hpp/core/path-optimization/cost.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      template <typename _Spline, int DerivativeOrder>
      struct HPP_CORE_LOCAL SquaredLength // : Cost
      {
        typedef _Spline Spline;
        typedef typename Spline::Ptr_t SplinePtr_t;
        typedef std::vector<SplinePtr_t> Splines_t;

        SquaredLength (const Splines_t& splines, size_type paramSize, size_type paramDerivativeSize)
          :
        // SplineCost (size_type nSplines, size_type paramSize, size_type paramDerivativeSize, const std::string& name)
          // : Cost (nSplines * Spline::NbCoeffs * paramSize, splines.size() * Spline::NbCoeffs * inputDerivativeSize, name),
          lambda_ (splines.size()),
          nSplines_ (splines.size()),
          paramSize_ (paramSize),
          paramDerivativeSize_ (paramDerivativeSize),
          inputSize_ (nSplines_ * Spline::NbCoeffs * paramSize),
          inputDerivativeSize_ (nSplines_ * Spline::NbCoeffs * paramDerivativeSize)
        {
          for (std::size_t i = 0; i < nSplines_; ++i)
            lambda_[i] = splines[i]->squaredNormIntegral(DerivativeOrder);
          value_type lMax = lambda_.maxCoeff();
          lambda_ = (lambda_.array() > 1e-6 * lMax)
            .select(lambda_.cwiseInverse(), 1e6 / lMax);
        }

        void value (value_type& result, const Splines_t& splines) const
        {
          assert (nSplines_ == splines.size());
          result = 0;
          for (std::size_t i = 0; i < nSplines_; ++i)
            result += lambda_[i] * splines[i]->squaredNormIntegral(DerivativeOrder);
        }

        void jacobian (vectorOut_t J, const Splines_t& splines) const
        {
          assert (nSplines_ == splines.size());
          assert (J.size() == inputDerivativeSize_);
          size_type col = 0;
          size_type size = Spline::NbCoeffs * paramDerivativeSize_;
          for (std::size_t i = 0; i < nSplines_; ++i) {
            splines[i]->squaredNormIntegralDerivative(DerivativeOrder,
                J.segment (col, size));
            J.segment (col, size) *= lambda_[i];
            col += size;
          }
        }

	void hessian (matrixOut_t H, const Splines_t& splines) const
        {
          assert (H.rows() == inputDerivativeSize_);
          assert (H.cols() == inputDerivativeSize_);
          typename Spline::BasisFunctionIntegralMatrix_t Ic;

          H.setZero();

          for (std::size_t k = 0; k < nSplines_; ++k) {
            splines[k]->squaredNormBasisFunctionIntegral (DerivativeOrder, Ic);
            Ic *= 2;
            const size_type shift = k * Spline::NbCoeffs * paramSize_;
            for (size_type i = 0; i < Spline::NbCoeffs; ++i) {
              for (size_type j = 0; j < Spline::NbCoeffs; ++j) {
                H.block(shift + i * paramSize_, shift + j * paramSize_, paramSize_, paramSize_)
                  .diagonal().setConstant (Ic(i,j) * lambda_[k]);
              }
            }

#ifndef NDEBUG
            value_type res1 = 0.5 * splines[k]->rowParameters().transpose() *
              H.block(shift, shift, Spline::NbCoeffs * paramSize_, Spline::NbCoeffs * paramSize_)
              * splines[k]->rowParameters();

            value_type res2 = splines[k]->squaredNormIntegral(DerivativeOrder) * lambda_[k];

            value_type diff = res1 - res2;

            if (std::fabs(diff) > Eigen::NumTraits<value_type>::dummy_precision()) {
              hppDout (error, "Hessian seems wrong for spline " << k << ": "
                  << res1 << " - " << res2 << " = " << res1 - res2 << '\n'
                  << H.block(shift, shift, Spline::NbCoeffs * paramSize_, Spline::NbCoeffs * paramSize_));
            }
#endif // NDEBUG
          }
        }

        vector_t lambda_;
        const std::size_t nSplines_;
        const size_type paramSize_, paramDerivativeSize_;
        const size_type inputSize_, inputDerivativeSize_;
      };
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH
