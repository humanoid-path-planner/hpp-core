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

# include <hpp/core/path/spline.hh>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>

namespace hpp {
  namespace core {
    namespace path {
      namespace internal {
        template <int N>
        inline Eigen::Matrix<size_type, N, 1> factorials ()
        {
          Eigen::Matrix<size_type, N, 1> ret;
          ret (0) = 1;
          for (size_type i = 1; i < N; ++i) ret(i) = ret(i-1) * i;
          return ret;
        }
        template <int N>
        inline size_type binomial (size_type n, size_type k, const Eigen::Matrix<size_type, N, 1> factors)
        {
          assert (n >= k && k >= 0);
          assert (n < N);
          size_type denom = factors(k) * factors(n - k);
          return factors (n) / denom;
        }

        template <int Degree>
        void spline_basis_function<CanonicalPolynomeBasis, Degree>::eval (const value_type t, Coeffs_t& res)
        {
          res(0) = 1;
          for (size_type i = 1; i < NbCoeffs; ++i) res(i) = res(i-1) * t;
        }
        template <int Degree>
        void spline_basis_function<CanonicalPolynomeBasis, Degree>::derivative
        (const size_type order, const value_type& t, Coeffs_t& res)
        {
          static Factorials_t factors = factorials<NbCoeffs>();
          value_type powerOfT = 1;
          res.head(order).setZero();
          for (size_type i = order; i < NbCoeffs; ++i) {
            res(i) = value_type(factors(i) / factors(i - order)) * powerOfT;
            powerOfT *= t;
          }
        }
        template <int Degree>
        void spline_basis_function<CanonicalPolynomeBasis, Degree>::integral
        (const size_type order, IntegralCoeffs_t& res)
        {
          static Factorials_t factors = factorials<NbCoeffs>();

          // TODO the output matrix is symmetric
          if (order > 0) {
            res.topRows (order).setZero();
            res.leftCols(order).setZero();
          }

          for (size_type i = order; i < NbCoeffs ; ++i) {
            // TODO size_type + cache this values.
            const size_type factor_i (factors(i) / factors(i - order));
            for (size_type j = order; j < NbCoeffs; ++j) {
              // TODO size_type + cache this values.
              const size_type factor_j (factors(j) / factors(j - order));
              const size_type power = i + j - 2*order + 1;
              res(i, j) = value_type(factor_i * factor_j) / value_type(power);
            }
          }
        }

        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::eval (const value_type t, Coeffs_t& res)
        {
          res(0) = 1;
          for (size_type i = 1; i < NbCoeffs; ++i) res(i) = res(i-1) * t;
        }
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::derivative
        (const size_type k, const value_type& t, Coeffs_t& res)
        {
          if (k > Degree) {
            res.setZero();
            return;
          }
          static Factorials_t factors = factorials<NbCoeffs>();
          Coeffs_t powersOfT;
          powersOfT(0) = 1;
          for (size_type i = 0; i < NbCoeffs; ++i) powersOfT(i) = powersOfT(i - 1) * t;

          for (size_type i = 0; i < NbCoeffs; ++i) {
            res(i) = 0;
            for (size_type p = 0; p <= std::min(i, k); ++p) {
              size_type ip = i - p;
              res(i) += ( k - p % 2 == 0 ? 1 : -1 )
                * binomial (k, p, factors)
                *   powersOfT(ip) * powersOfT(Degree - k - ip)
                / ( factors  (ip) * factors  (Degree - k - ip) );
            }
          }
          res *= factors(Degree);
        }
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::integral
        (const size_type order, IntegralCoeffs_t& res)
        {
          throw std::logic_error("Unimplemented");
        }
      }

      template <int _SplineType, int _Order>
      std::ostream& Spline<_SplineType, _Order>::print (std::ostream& os) const
      {
        os << "Spline (type=" << PolynomeBasis << ", order=" << Order
          << ")\nbase = " << base_.transpose()
          << '\n' << parameters_ << std::endl;
        return os;
      }

      template <int _SplineType, int _Order>
      bool Spline<_SplineType, _Order>::impl_compute (ConfigurationOut_t res, value_type t) const
      {
        BasisFunctionVector_t basisFunc;
        basisFunctionDerivative(0, t, basisFunc);
        velocity_.noalias() = parameters_.transpose() * basisFunc;

        pinocchio::integrate<false, hpp::pinocchio::LieGroupTpl> (robot_, base_, velocity_, res);
        return true;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_derivative (vectorOut_t res, const value_type& t, size_type order) const
      {
        assert (order > 0);
        BasisFunctionVector_t basisFunc;
        basisFunctionDerivative(order, t, basisFunc);
        res.noalias() = parameters_.transpose() * basisFunc;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_paramDerivative (vectorOut_t res, const value_type& t) const
      {
        BasisFunctionVector_t basisFunc;
        basisFunctionDerivative(0, t, basisFunc);
        res = basisFunc;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_paramIntegrate (vectorIn_t dParam)
      {
        pinocchio::integrate<false, hpp::pinocchio::LieGroupTpl>
          (robot_, base_, dParam.head(robot_->numberDof()), base_);

        Eigen::Map<vector_t, Eigen::Aligned> (parameters_.data(), parameters_.size())
          .noalias() += dParam;
      }

      template class Spline<CanonicalPolynomeBasis, 1>; // equivalent to StraightPath
      template class Spline<CanonicalPolynomeBasis, 2>;
      template class Spline<CanonicalPolynomeBasis, 3>;
      template class Spline<BernsteinBasis, 1>; // equivalent to StraightPath
      template class Spline<BernsteinBasis, 2>;
      template class Spline<BernsteinBasis, 3>;
    } //   namespace path
  } //   namespace core
} // namespace hpp
