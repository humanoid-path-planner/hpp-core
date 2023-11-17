// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH

#include <hpp/core/path-optimization/cost.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
/// TODO
/// The derivative of the cost is wrong when for freeflyer and planar
/// joints. It lacks the derivative of the difference operator. The issue
/// is that it is not a quadratic cost anymore.
template <typename _Spline>
struct HPP_CORE_LOCAL L2NormSquaredOfDerivative {
  typedef _Spline Spline;
  typedef typename Spline::Ptr_t SplinePtr_t;
  typedef std::vector<SplinePtr_t> Splines_t;

  L2NormSquaredOfDerivative(const Splines_t& splines, size_type paramSize,
                            size_type paramDerivativeSize,
                            size_type derivativeOrder)
      : lambda_(splines.size()),
        nSplines_(splines.size()),
        paramSize_(paramSize),
        paramDerivativeSize_(paramDerivativeSize),
        inputSize_(nSplines_ * Spline::NbCoeffs * paramSize),
        inputDerivativeSize_(nSplines_ * Spline::NbCoeffs *
                             paramDerivativeSize),
        derivativeOrder_(derivativeOrder) {
    assert(derivativeOrder_ > 0);
    // Spline::NbPowerOfT = 2 * Spline::Order + 3
    if (2 * derivativeOrder_ - 1 >= Spline::NbPowerOfT) {
      HPP_THROW(std::invalid_argument,
                "Cannot compute the squared norm of the "
                    << derivativeOrder_
                    << "th order derivative with splines of order "
                    << Spline::Order);
    }
    lambda_.setOnes();
  }

  void computeLambdasFromSplineLength(const Splines_t& splines) {
    for (std::size_t i = 0; i < nSplines_; ++i)
      lambda_[i] = splines[i]->squaredNormIntegral(derivativeOrder_);
    value_type lMax = lambda_.maxCoeff();
    // Make sure there is no too relatively small values in lambda_.
    lambda_ = (lambda_.array() > 1e-6 * lMax)
                  .select(lambda_.cwiseInverse(), 1e6 / lMax);
  }

  void value(value_type& result, const Splines_t& splines) const {
    assert(nSplines_ == splines.size());
    result = 0;
    for (std::size_t i = 0; i < nSplines_; ++i)
      result += lambda_[i] * splines[i]->squaredNormIntegral(derivativeOrder_);
  }

  void jacobian(vectorOut_t J, const Splines_t& splines) const {
    assert(nSplines_ == splines.size());
    assert(J.size() == inputDerivativeSize_);
    size_type col = 0;
    size_type size = Spline::NbCoeffs * paramDerivativeSize_;
    for (std::size_t i = 0; i < nSplines_; ++i) {
      splines[i]->squaredNormIntegralDerivative(derivativeOrder_,
                                                J.segment(col, size));
      J.segment(col, size) *= lambda_[i];
      col += size;
    }
  }

  void hessian(matrixOut_t H, const Splines_t& splines) const {
    assert(H.rows() == inputDerivativeSize_);
    assert(H.cols() == inputDerivativeSize_);
    typename Spline::BasisFunctionIntegralMatrix_t Ic;

    H.setZero();

    for (std::size_t k = 0; k < nSplines_; ++k) {
      splines[k]->squaredNormBasisFunctionIntegral(derivativeOrder_, Ic);
      Ic *= 2;
      const size_type shift = k * Spline::NbCoeffs * paramSize_;
      for (size_type i = 0; i < Spline::NbCoeffs; ++i) {
        for (size_type j = 0; j < Spline::NbCoeffs; ++j) {
          H.block(shift + i * paramSize_, shift + j * paramSize_, paramSize_,
                  paramSize_)
              .diagonal()
              .setConstant(Ic(i, j) * lambda_[k]);
        }
      }

#ifndef NDEBUG
      value_type res1 = 0.5 * splines[k]->rowParameters().transpose() *
                        H.block(shift, shift, Spline::NbCoeffs * paramSize_,
                                Spline::NbCoeffs * paramSize_) *
                        splines[k]->rowParameters();

      value_type res2 =
          splines[k]->squaredNormIntegral(derivativeOrder_) * lambda_[k];

      value_type diff = res1 - res2;

      if (std::fabs(diff) > Eigen::NumTraits<value_type>::dummy_precision()) {
        hppDout(error,
                "Hessian seems wrong for spline "
                    << k << ": " << res1 << " - " << res2 << " = "
                    << res1 - res2 << '\n'
                    << H.block(shift, shift, Spline::NbCoeffs * paramSize_,
                               Spline::NbCoeffs * paramSize_));
      }
#endif  // NDEBUG
    }
  }

  vector_t lambda_;
  const std::size_t nSplines_;
  const size_type paramSize_, paramDerivativeSize_;
  const size_type inputSize_, inputDerivativeSize_;
  size_type derivativeOrder_;
};
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH
