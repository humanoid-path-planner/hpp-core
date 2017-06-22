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

        /// Spline basis functions input set is [0, 1]
        template <int Degree> struct spline_basis_function <CanonicalPolynomeBasis, Degree>
        {
          typedef sbf_traits<CanonicalPolynomeBasis, Degree> traits;
          enum { NbCoeffs = traits::NbCoeffs };
          typedef Eigen::Matrix<size_type, NbCoeffs, 1> Factorials_t;
          typedef typename traits::Coeffs_t Coeffs_t;
          typedef typename traits::IntegralCoeffs_t IntegralCoeffs_t;

          static void eval (const value_type t, Coeffs_t& res);
          static void derivative (const size_type order, const value_type& t, Coeffs_t& res);
          /// Integrate between 0 and 1
          static void integral (const size_type order, IntegralCoeffs_t& res);
        };
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

        template <int Degree> struct spline_basis_function <BernsteinBasis, Degree>
        {
          enum { NbCoeffs = Degree + 1 };
          typedef Eigen::Matrix<size_type, NbCoeffs, 1> Factorials_t;
          typedef Eigen::Matrix<value_type, NbCoeffs, 1> Coeffs_t;
          typedef Eigen::Matrix<value_type, NbCoeffs, NbCoeffs> IntegralCoeffs_t;

          static void eval (const value_type t, Coeffs_t& res);
          static void derivative (const size_type order, const value_type& t, Coeffs_t& res);
          /// Integrate between 0 and 1
          static void integral (const size_type order, IntegralCoeffs_t& res);
        };
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::eval (const value_type t, Coeffs_t& res)
        {
          derivative(0, t, res);
        }
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::derivative
        (const size_type k, const value_type& t, Coeffs_t& res)
        {
          res.setZero();
          if (k > Degree) return;
          static Factorials_t factors = factorials<NbCoeffs>();
          Coeffs_t powersOfT, powersOfOneMinusT;
          powersOfT        (0) = 1;
          powersOfOneMinusT(0) = 1;
          const value_type oneMinusT = 1 - t;
          for (size_type i = 1; i < NbCoeffs; ++i) {
            powersOfT        (i) = powersOfT        (i - 1) * t;
            powersOfOneMinusT(i) = powersOfOneMinusT(i - 1) * oneMinusT;
          }

          for (size_type i = 0; i < NbCoeffs; ++i) {
            for (size_type p = std::max((size_type)0, k + i - Degree); p <= std::min(i, k); ++p) {
              size_type ip = i - p;
              res(i) += value_type (( (k - p) % 2 == 0 ? 1 : -1 )
                * binomial (k, p, factors))
                *   powersOfT(ip) * powersOfOneMinusT(Degree - k - ip)
                / value_type( factors  (ip) * factors  (Degree - k - ip) );
            }
          }
          res *= value_type(factors(Degree));
        }
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::integral
        (const size_type k, IntegralCoeffs_t& res)
        {
          res.setZero();
          if (k > Degree) return;
          static Eigen::Matrix<size_type, 2*NbCoeffs - 1, 1> factors = factorials<2*NbCoeffs-1>();

          Factorials_t among_k, among_n_minus_k;
          for (size_type i = 0; i <= k; ++i) among_k(i) = binomial(k, i, factors);
          for (size_type i = 0; i <= Degree-k; ++i) among_n_minus_k(i) = binomial(Degree-k, i, factors);

          for (size_type i = 0; i < NbCoeffs; ++i) {
            size_type I_i_min = std::max((size_type)0, k + i - Degree);
            size_type I_i_max = std::min(i, k);

            for (size_type j = 0; j < NbCoeffs; ++j) {
              size_type I_j_min = std::max((size_type)0, k + j - Degree);
              size_type I_j_max = std::min(j, k);

              for (size_type p = I_i_min; p <= I_i_max; ++p) {
                for (size_type q = I_j_min; q <= I_j_max; ++q) {
                  value_type alpha_0 =
                    value_type (among_n_minus_k (i-p) * among_n_minus_k (j-q))
                    / value_type(
                        binomial(2*(Degree-k), i-p+j-q, factors)
                        * (2*(Degree-k) + 1)
                      );

                  res(i,j) += value_type(
                      ( (2*k - p - q) % 2 == 0 ? 1 : -1 )
                      * among_k (p)
                      * among_k (q))
                    * alpha_0;
                }
              }
            }
          }
          res *= value_type(factors(Degree) / factors(Degree - k));
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
      void Spline<_SplineType, _Order>::basisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res) const
      {
        // TODO: add a cache.
        assert (u >= 0 && u <= 1);
        BasisFunction_t::derivative (order, u, res);
        res /= powersOfT_(order);
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::squaredNormBasisFunctionIntegral (const size_type order, BasisFunctionIntegralMatrix_t& Ic) const
      {
        // TODO: add a cache.
        BasisFunction_t::integral (order, Ic);
        if (order > 0) Ic /= powersOfT_[2 * order - 1];
        else           Ic *= powersOfT_[1];
      }

      template <int _SplineType, int _Order>
      bool Spline<_SplineType, _Order>::impl_compute (ConfigurationOut_t res, value_type t) const
      {
        BasisFunctionVector_t basisFunc;
        const value_type u = (t - timeRange().first) / timeRange().second;
        basisFunctionDerivative(0, u, basisFunc);
        velocity_.noalias() = parameters_.transpose() * basisFunc;

        pinocchio::integrate<false, hpp::pinocchio::LieGroupTpl> (robot_, base_, velocity_, res);
        return true;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_derivative (vectorOut_t res, const value_type& t, size_type order) const
      {
        assert (order > 0);
        BasisFunctionVector_t basisFunc;
        const value_type u = (t - timeRange().first) / timeRange().second;
        basisFunctionDerivative(order, u, basisFunc);
        res.noalias() = parameters_.transpose() * basisFunc;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_paramDerivative (vectorOut_t res, const value_type& t) const
      {
        BasisFunctionVector_t basisFunc;
        const value_type u = (t - timeRange().first) / timeRange().second;
        basisFunctionDerivative(0, u, basisFunc);
        res = basisFunc;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_paramIntegrate (vectorIn_t dParam)
      {
        // pinocchio::integrate<false, hpp::pinocchio::LieGroupTpl>
          // (robot_, base_, dParam.head(robot_->numberDof()), base_);

        ParameterVector_t(parameters_.data(), parameters_.size())
          .noalias() += dParam;
      }

      template <int _SplineType, int _Order>
      value_type Spline<_SplineType, _Order>::squaredNormIntegral (const size_type order)
      {
        typename sbf_traits::IntegralCoeffs_t Ic;
        squaredNormBasisFunctionIntegral(order, Ic);
        return (parameters_ * parameters_.transpose()).cwiseProduct(Ic).sum();
        // return (parameters_.transpose() * (Ic * parameters_)).trace();
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::squaredNormIntegralDerivative (const size_type order, vectorOut_t res)
      {
        typename BasisFunction_t::IntegralCoeffs_t Ic;
        squaredNormBasisFunctionIntegral(order, Ic);
        matrix_t tmp (parameters_.transpose() * Ic);
        res = 2 * Eigen::Map<vector_t, Eigen::Aligned> (tmp.data(), tmp.size());
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
