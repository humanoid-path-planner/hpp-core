// Copyright (c) 2017 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_SPLINE_HH
# define HPP_CORE_PATH_SPLINE_HH

# include <hpp/core/path.hh>

# include <hpp/pinocchio/device.hh>

# include <hpp/constraints/matrix-view.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    namespace path {
      /// \addtogroup path
      /// \{

      enum PolynomeBasisType {
        CanonicalPolynomeBasis,
        BernsteinBasis
      };

      /// \cond
      namespace internal {
        template <int SplineType, int Degree> struct spline_basis_function;
        template <int SplineType, int Degree> struct sbf_traits {
          enum { NbCoeffs = Degree + 1 };
          typedef Eigen::Matrix<value_type, NbCoeffs, 1> Coeffs_t;
          typedef Eigen::Matrix<value_type, NbCoeffs, NbCoeffs> IntegralCoeffs_t;
        };
      }
      /// \endcond

      /// Base class for spline paths
      template <int _PolynomeBasis, int _Order>
      class HPP_CORE_DLLAPI Spline : public Path
      {
        public:
          enum {
            PolynomeBasis = _PolynomeBasis,
            Order = _Order,
            NbCoeffs = _Order + 1,
            NbPowerOfT = 2 * NbCoeffs + 1
          };

          typedef internal::sbf_traits<PolynomeBasis, Order> sbf_traits;
          typedef internal::spline_basis_function<PolynomeBasis, Order> BasisFunction_t;
          typedef Eigen::Matrix<value_type, NbPowerOfT, 1> PowersOfT_t;
          typedef typename sbf_traits::Coeffs_t BasisFunctionVector_t;
          typedef typename sbf_traits::IntegralCoeffs_t BasisFunctionIntegralMatrix_t;

          typedef Eigen::Matrix<value_type, NbCoeffs, Eigen::Dynamic, Eigen::RowMajor> ParameterMatrix_t;
          typedef Eigen::Map<const vector_t, Eigen::Aligned> ConstParameterVector_t;
          typedef Eigen::Map<      vector_t, Eigen::Aligned>      ParameterVector_t;

          typedef boost::shared_ptr<Spline> Ptr_t;
          typedef boost::weak_ptr<Spline> WkPtr_t;

          size_type parameterSize () const
          {
            return parameterSize_;
          }

          /** The partial derivative with respects to the parameters is of the form
          /// \f{eqnarray*}{
          /// \frac{\partial S}{\partial p_{k}} (q, p, t)    &=& B_k(t) \times I \\
          /// \frac{\partial S}{\partial q_{base}} (q, p, t) &=& I
          /// \f}
          /// This method returns the coefficients \f$ (B_k(t))_{k} \f$
          **/
          void parameterDerivativeCoefficients (vectorOut_t res, const value_type& t) const
          {
            assert (res.size() == NbCoeffs);
            impl_paramDerivative (res, t);
          }

          void parameterIntegrate (vectorIn_t dParam)
          {
            assert (dParam.size() == NbCoeffs * parameterSize_);
            impl_paramIntegrate (dParam);
          }

          value_type squaredNormIntegral (const size_type order);

          void squaredNormIntegralDerivative (const size_type order, vectorOut_t res);

          void basisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res) const;

          void basisFunctionDerivative (const size_type order, const value_type& u, vectorOut_t res) const
          {
            assert (res.size() == NbCoeffs);
            BasisFunctionVector_t tmp;
            basisFunctionDerivative(order, u, tmp);
            res = tmp;
          }

          void maxVelocity (vectorOut_t res) const;

          void squaredNormBasisFunctionIntegral (const size_type order, BasisFunctionIntegralMatrix_t& res) const;

          void squaredNormBasisFunctionIntegral (const size_type order, matrixOut_t res) const
          {
            // assert (res.size() == NbCoeffs);
            BasisFunctionIntegralMatrix_t tmp;
            squaredNormBasisFunctionIntegral (order, tmp);
            res = tmp;
          }

          Configuration_t initial () const
          {
            Configuration_t q (outputSize());
            bool res = operator() (q, timeRange().first);
            assert(res);
            return q;
          }

          Configuration_t end () const
          {
            Configuration_t q (outputSize());
            bool res = operator() (q, timeRange().second);
            assert(res);
            return q;
          }

          const Configuration_t& base () const
          {
            return base_;
          }

          void base (const Configuration_t& q)
          {
            base_ = q;
          }

          /// Each row corresponds to a velocity of the robot.
          const ParameterMatrix_t& parameters () const
          {
            return parameters_;
          }

          void parameters (const ParameterMatrix_t& m)
          {
            parameters_ = m;
          }

          ConstParameterVector_t rowParameters () const
          {
            return ConstParameterVector_t (parameters_.data(), parameters_.size());
          }

          void rowParameters (vectorIn_t p)
          {
            ParameterVector_t(parameters_.data(), parameters_.size()) = p;
          }

          PathPtr_t copy () const
          {
            Ptr_t other (new Spline (*this));
            other->initCopy(other);
            return other;
          }

          PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
          {
            Ptr_t other (new Spline (*this, constraints));
            other->initCopy(other);
            return other;
          }

          virtual ~Spline () throw () {}

          static Ptr_t create (const DevicePtr_t& robot,
              const interval_t& interval,
              const ConstraintSetPtr_t& constraints)
          {
            Ptr_t shPtr (new Spline(robot, interval, constraints));
            shPtr->init(shPtr);
            return shPtr;
          }

        protected:
          Spline (const DevicePtr_t& robot,
              const interval_t& interval,
              const ConstraintSetPtr_t& constraints)
            : Path (interval, robot->configSize(), robot->numberDof(), constraints),
            parameterSize_ (robot->numberDof()),
            robot_ (robot),
            base_ (outputSize()),
            parameters_ ((int)NbCoeffs, parameterSize_)
          {
            powersOfT_(0) = 1;
            for (size_type i = 1; i < NbPowerOfT; ++i)
              powersOfT_(i) = powersOfT_(i - 1) * length();
          }

          Spline (const Spline& path);

          Spline (const Spline& path, const ConstraintSetPtr_t& constraints);

          void init (const Ptr_t& self) { Path::init(self); weak_ = self; }

          void initCopy (const Ptr_t& self) { Path::initCopy(self); weak_ = self; }

          std::ostream& print (std::ostream &os) const;

          bool impl_compute (ConfigurationOut_t configuration, value_type t) const;

          void impl_derivative (vectorOut_t res, const value_type& t, size_type order) const;

          void impl_paramDerivative (vectorOut_t res, const value_type& t) const;

          void impl_paramIntegrate (vectorIn_t dParam);

          size_type parameterSize_;
          DevicePtr_t robot_;
          Configuration_t base_;
          ParameterMatrix_t parameters_;

        private:
          WkPtr_t weak_;

          mutable vector_t velocity_;
          mutable PowersOfT_t powersOfT_;
      }; // class Spline
      /// \}
    } //   namespace path
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_SPLINE_HH
