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
#include <hpp/pinocchio/liegroup-space.hh>

#include <path/math.hh>

namespace hpp {
  namespace core {
    namespace path {
      namespace internal {
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
          static void absDerBounds (Coeffs_t& res);
          static void bound (const size_type& order, const value_type& /* t0 */, const value_type& t1, Coeffs_t& res) { derivative(order, t1, res); }
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
          static Factorials_t factors = binomials<NbCoeffs>::factorials();
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
          static Factorials_t factors = binomials<NbCoeffs>::factorials();

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
        void spline_basis_function<CanonicalPolynomeBasis, Degree>::absDerBounds
        (Coeffs_t& res)
        {
          res(0) = 0;
          for (size_type i = 1; i < NbCoeffs; ++i) res(i) = value_type(i);
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
          static void absDerBounds (Coeffs_t& res);
          static void bound (const size_type& order, const value_type& t0, const value_type& t1, Coeffs_t& res);

          private:
          static Coeffs_t absBound (bool up);
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
          static Factorials_t factors = binomials<NbCoeffs>::factorials();
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
                * binomials<NbCoeffs>::binomial (k, p))
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
          const size_type N = 2*NbCoeffs-1;
          static const typename binomials<N>::Factorials_t& factors = binomials<2*NbCoeffs-1>::factorials();

          Factorials_t among_k, among_n_minus_k;
          for (size_type i = 0; i <= k; ++i) among_k(i) = binomials<N>::binomial(k, i);
          for (size_type i = 0; i <= Degree-k; ++i) among_n_minus_k(i) = binomials<N>::binomial(Degree-k, i);

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
                        binomials<N>::binomial(2*(Degree-k), i-p+j-q)
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
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::absDerBounds
        (Coeffs_t& res)
        {
          res.setConstant (2*Degree);
        }
        template <int Degree>
        void spline_basis_function<BernsteinBasis, Degree>::bound
        (const size_type& order, const value_type& t0, const value_type& t1, Coeffs_t& res)
        {
          (void)order; // Suppress unused warning when NDEBUG
          assert(order == 1);
          static const Coeffs_t b_up = absBound(true);
          static const Coeffs_t b_um = absBound(false);
          Coeffs_t bt0, bt1;
          derivative(1, t0, bt0);
          derivative(1, t1, bt1);

          res.noalias() = bt0.cwiseAbs().cwiseMax(bt1.cwiseAbs());

          // Case i = 0 and i = n
          // Nothing to do.

          // Case i = 1, n-1
          if (t0 * Degree < 2 && 2 < t1 * Degree) // If max for i = 1 in [t0, t1]
            res(1) = std::max(res(1), b_up(1));
          if (t0 * Degree < Degree - 2 && Degree - 2 < t1 * Degree) // If max for i = n-1 in [t0, t1]
            res(Degree-1) = std::max(res(Degree-1), b_up(Degree-1));

          // Case 2 <= i <= n-2
          if (Degree > 3) {
            // when u_p(i) in [t0, t1], consider b_up.
            size_type r1 = std::max(size_type(std::ceil (t0 * (Degree - 1))), size_type(2)),
                      r2 =          size_type(std::floor(t1 * (Degree - 1))),
                      nr = std::max(r2 - r1 + 1, size_type(0));
            res.segment(r1, nr).noalias() = res.segment(r1, nr).cwiseMax(b_up.segment(r1, nr));

            // when u_m(i) in [t0, t1], consider b_um.
            r1 =          size_type(std::ceil (1 + t0 * (Degree - 1))),
            r2 = std::min(size_type(std::floor(1 + t1 * (Degree - 1))), size_type(Degree - 2));
            nr = std::max(r2 - r1 + 1, size_type(0));
            res.segment(r1, nr).noalias() = res.segment(r1, nr).cwiseMax(b_um.segment(r1, nr));
          }
        }
        template <int Degree>
        typename spline_basis_function<BernsteinBasis, Degree>::Coeffs_t spline_basis_function<BernsteinBasis, Degree>::absBound (bool up)
        {
          Coeffs_t res;
          Factorials_t iToThePowerOfI (Factorials_t::Ones());
          for (size_type i = 1; i < Degree; ++i) {
            for (size_type j = 0; j < i; ++j)
              iToThePowerOfI(i) *= i;
          }
          res(0) = res(Degree) = Degree;
          if (Degree > 1) {
            if (Degree == 2) res(1) = res(Degree - 1) = 2;
            else res(1) = res(Degree - 1)
              = value_type(iToThePowerOfI(Degree - 2)) / std::pow (Degree, Degree-3);
            for (size_type i = 2; i < Degree - 1; ++i) {
              const size_type p = (up
                  ? iToThePowerOfI(i) * iToThePowerOfI(Degree - i - 1)
                  : iToThePowerOfI(Degree - i) * iToThePowerOfI(i - 1));
              res(i) = value_type(binomials<NbCoeffs>::binomial(Degree, i) * p) / value_type(iToThePowerOfI(Degree - 1));
            }
          }
          return res;
        }
      }

      template <int _SplineType, int _Order>
      std::ostream& Spline<_SplineType, _Order>::print (std::ostream& os) const
      {
        os << "Spline (type=" << PolynomeBasis << ", order=" << Order
          << ")\nbase = " << base().transpose()
          << '\n' << parameters_ << std::endl;
        return os;
      }

      template <int _SplineType, int _Order>
      Spline<_SplineType, _Order>::Spline (const Spline& path) :
        Path (path),
        parameterSize_ (path.parameterSize_),
        robot_ (path.robot_),
        base_ (path.base_),
        parameters_ (path.parameters_),
        velocity_ (path.velocity_),
        powersOfT_ (path.powersOfT_)
      {}

      template <int _SplineType, int _Order>
      Spline<_SplineType, _Order>::Spline (const Spline& path, const ConstraintSetPtr_t& constraints) :
        Path (path, constraints),
        parameterSize_ (path.parameterSize_),
        robot_ (path.robot_),
        base_ (path.base_),
        parameters_ (path.parameters_),
        velocity_ (path.velocity_),
        powersOfT_ (path.powersOfT_)
      {}

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::timeFreeBasisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res)
      {
        // TODO: add a cache.
        assert (u >= 0 && u <= 1);
        BasisFunction_t::derivative (order, u, res);
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::basisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res) const
      {
        // TODO: add a cache.
        assert (u >= 0 && u <= 1);
        if (length() == 0) res.setZero();
        else {
          BasisFunction_t::derivative (order, u, res);
          res /= powersOfT_(order);
        }
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::maxVelocity (vectorOut_t res) const
      {
        BasisFunctionVector_t ub;
        BasisFunction_t::absDerBounds (ub);
        ub /= length();
        res.transpose() = ub.transpose() * parameters_.cwiseAbs();
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::squaredNormBasisFunctionIntegral (const size_type order, BasisFunctionIntegralMatrix_t& Ic) const
      {
        // TODO: add a cache.
        if (length() == 0) Ic.setZero();
        else {
          BasisFunction_t::integral (order, Ic);
          if (order > 0) Ic /= powersOfT_[2 * order - 1];
          else           Ic *= powersOfT_[1];
        }
      }

      template <int _SplineType, int _Order>
      bool Spline<_SplineType, _Order>::impl_compute (ConfigurationOut_t res, value_type s) const
      {
        const value_type u = (length() == 0 ? 0 : (s - paramRange().first) / paramLength());
        value (base_, parameters_, u, res, velocity_);
        return true;
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_derivative (vectorOut_t res, const value_type& s, size_type order) const
      {
        // p = q + v(t) so dp/dt = d+/dv * dv/dt
        assert (order > 0);
        // For non vector space, it is not possible to compute the derivatives
        // at a higher order. At the d2+/dv2 is not available for SE(n) and SO(n).
        assert (order == 1 || robot_->configSpace()->isVectorSpace());
        BasisFunctionVector_t basisFunc;
        const value_type u = (length() == 0 ? 0 : (s - paramRange().first) / paramLength());
        basisFunctionDerivative(order, u, basisFunc);
        res.noalias() = parameters_.transpose() * basisFunc;

        if (!robot_->configSpace()->isVectorSpace()) {
          basisFunctionDerivative(0, u, basisFunc);
          vector_t v (parameters_.transpose() * basisFunc);
          // true means: res <- Jdiff * res
          base_.space()->dIntegrate_dv<pinocchio::DerivativeTimesInput> (base_, v, res);
        }
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::impl_paramDerivative (vectorOut_t res, const value_type& s) const
      {
        BasisFunctionVector_t basisFunc;
        const value_type u = (length() == 0 ? 0 : (s - paramRange().first) / paramLength());
        basisFunctionDerivative(0, u, basisFunc);
        res = basisFunc;

        if (!robot_->configSpace()->isVectorSpace()) {
          vector_t v (parameters_.transpose() * basisFunc);
          matrix_t unused;
          // true means: res <- Jdiff * res
          base_.space()->Jdifference<true> (base(), v, unused, res);
        }
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
      void Spline<_SplineType, _Order>::impl_velocityBound (
          vectorOut_t res, const value_type& t0, const value_type& t1) const
      {
        BasisFunctionVector_t ub;
        BasisFunction_t::bound (1, t0, t1, ub);
        ub /= length();
        res.noalias() = parameters_.cwiseAbs().transpose() * ub;
      }

      template <int _SplineType, int _Order>
      value_type Spline<_SplineType, _Order>::squaredNormIntegral (const size_type order) const
      {
        typename sbf_traits::IntegralCoeffs_t Ic;
        squaredNormBasisFunctionIntegral(order, Ic);
        return (parameters_ * parameters_.transpose()).cwiseProduct(Ic).sum();
        // return (parameters_.transpose() * (Ic * parameters_)).trace();
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::squaredNormIntegralDerivative (const size_type order, vectorOut_t res) const
      {
        typename BasisFunction_t::IntegralCoeffs_t Ic;
        squaredNormBasisFunctionIntegral(order, Ic);
        matrix_t tmp (parameters_.transpose() * Ic);
        res = 2 * Eigen::Map<vector_t, Eigen::Aligned> (tmp.data(), tmp.size());
      }

      template <int _SplineType, int _Order>
      void Spline<_SplineType, _Order>::value (pinocchio::LiegroupElementConstRef base,
          Eigen::Ref<const ParameterMatrix_t> params, const value_type& u,
          ConfigurationOut_t res, vectorOut_t velocity)
      {
        assert (0 <= u && u <= 1);
        assert (params.rows() == NbCoeffs);
        assert (params.cols() == base.space()->nv());

        velocity.resize (base.space()->nv());

        BasisFunctionVector_t basisFunc;
        BasisFunction_t::derivative (0, u, basisFunc);
        velocity.noalias() = params.transpose() * basisFunc;

        res = (base + velocity).vector();
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
