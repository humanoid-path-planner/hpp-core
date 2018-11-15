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
# include <hpp/pinocchio/liegroup-element.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/steering-method/fwd.hh>
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
      ///
      /// Splines are polynomials with various possible representations.
      /// \param _PolynomeBasis basis of polynomials used among
      ///    \li CanonicalPolynomeBasis for the canonical basis,
      ///    \li BernsteinBasis for Bernstein basis
      /// \param _Order degree of the polynomial representation.
      /// \sa hpp::core::path::PolynomeBasisType.
      ///
      /// Splines represent a curve in the tangent space of a given
      /// robot (hpp::core::Device) at a configuration called \b base.
      ///
      /// \f{eqnarray*}
      /// spline (u) &=& base + PM^{T} B (u)
      /// \f}
      ///
      /// where*
      /// \li \f$u\in [0,1]\f$,
      /// \li operator "+" should be understood as Lie group integration,
      /// \li \f$PM\f$ is the matrix of parameters the rows of
      /// which are the spline control points. This matrix is
      /// accessible via setter and getter Spline::parameters,
      /// \li \f$B (t)\f$ is the vector containing the values of the basis
      /// functions at parameter \f$t\f$.
      ///
      /// The dimension of control points, corresponding to the robot number of
      /// degrees of freedom can be retrieved by getter Spline::parameterSize.
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
           *  \f{eqnarray*}{
           *  \frac{\partial S}{\partial p_{k}} (q, p, t)    &=& B_k(t) \times I \\
           *  \frac{\partial S}{\partial q_{base}} (q, p, t) &=& I
           *  \f}
           *  This method returns the coefficients \f$ (B_k(t))_{k} \f$
          **/
          void parameterDerivativeCoefficients (vectorOut_t res, const value_type& t) const
          {
            assert (res.size() == NbCoeffs);
            impl_paramDerivative (res, t);
          }

          /// Adds dParam to the parameters
          void parameterIntegrate (vectorIn_t dParam)
          {
            assert (dParam.size() == NbCoeffs * parameterSize_);
            impl_paramIntegrate (dParam);
          }

          /// Returns \f$ \int S^{(k)}(t)^T \times S^{(k)}(t) dt \f$
          ///
          /// where k is the argument
          value_type squaredNormIntegral (const size_type order) const;

          /// Returns the derivative of \ref squaredNormIntegral wrt the parameters.
          ///
          /// \f[ res(j) \gets 2 \sum_i P_i^T \times m_{i,j} \f]
          void squaredNormIntegralDerivative (const size_type order, vectorOut_t res) const;

          /** Returns a vector \f$ (v_i) \f$ as
           *  \f[
           *  v_i = b_i^{(k)}(u)
           *  \f]
          **/
          static void timeFreeBasisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res);

          static void timeFreeBasisFunctionDerivative (const size_type order, const value_type& u, vectorOut_t res)
          {
            assert (res.size() == NbCoeffs);
            BasisFunctionVector_t tmp;
            timeFreeBasisFunctionDerivative(order, u, tmp);
            res = tmp;
          }

          /** Returns a vector \f$ (v_i) \f$ as
           *  \f[
           *  v_i = T^{-k} b_i^{(k)}(u)
           *  \f]
          **/
          void basisFunctionDerivative (const size_type order, const value_type& u, BasisFunctionVector_t& res) const;

          void basisFunctionDerivative (const size_type order, const value_type& u, vectorOut_t res) const
          {
            assert (res.size() == NbCoeffs);
            BasisFunctionVector_t tmp;
            basisFunctionDerivative(order, u, tmp);
            res = tmp;
          }

          /// Returns an upper bound of the velocity on the complete interval.
          /// \sa Path::velocityBound
          void maxVelocity (vectorOut_t res) const;

          /** Returns a matrix \f$ (m_{i,j}) \f$ as
           *  \f[
           *  m_{i,j} = T^{1-2k} \int_0^1 b_i^{(k)}(u) b_j^{(k)}(u) du
           *  \f]
          **/
          void squaredNormBasisFunctionIntegral (const size_type order, BasisFunctionIntegralMatrix_t& res) const;

          void squaredNormBasisFunctionIntegral (const size_type order, matrixOut_t res) const
          {
            // assert (res.size() == NbCoeffs);
            BasisFunctionIntegralMatrix_t tmp;
            squaredNormBasisFunctionIntegral (order, tmp);
            res = tmp;
          }

          virtual Configuration_t initial () const
          {
            Configuration_t q (outputSize());
            bool res = operator() (q, timeRange().first);
            assert(res); (void)res;
            return q;
          }

          virtual Configuration_t end () const
          {
            Configuration_t q (outputSize());
            bool res = operator() (q, timeRange().second);
            assert(res); (void)res;
            return q;
          }

          /// Get the base configuration.
          /// The parameters are velocities to be integrated from this
          /// configuration.
          const Configuration_t& base () const
          {
            return base_.vector();
          }

          /// \sa base() const
          void base (const Configuration_t& q)
          {
            base_.vector() = q;
          }

          /// Each row corresponds to a velocity of the robot.
          const ParameterMatrix_t& parameters () const
          {
            return parameters_;
          }

          /// Returns the \f$ (P_i^T) \f$.
          /// Each row contains one parameter.
          void parameters (const ParameterMatrix_t& m)
          {
            parameters_ = m;
          }

          /// Concatenate the parameters as one vector (P_0^T, ..., P_n^T).
          ConstParameterVector_t rowParameters () const
          {
            return ConstParameterVector_t (parameters_.data(), parameters_.size());
          }

          /// Set the parameters
          void rowParameters (vectorIn_t p)
          {
            ParameterVector_t(parameters_.data(), parameters_.size()) = p;
          }

          PathPtr_t copy () const
          {
            Ptr_t other (new Spline (*this));
            other->init(other);
            return other;
          }

          PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
          {
            Ptr_t other (new Spline (*this, constraints));
            other->init(other);
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

          /// Evaluate a spline.
          /// \param base   the base configuration.
          /// \param params concatenation of row vectors representing the
          ///               velocity interpolation points.
          /// \param u      the ratio, between 0 and 1.
          /// \retval config the output configuration
          /// \retval velocity the interpolated velocity
          static void value (pinocchio::LiegroupElementConstRef base,
              Eigen::Ref<const ParameterMatrix_t> params,
              const value_type& u,
              ConfigurationOut_t config,
              vectorOut_t velocity);

        protected:
          Spline (const DevicePtr_t& robot,
              const interval_t& interval,
              const ConstraintSetPtr_t& constraints)
            : Path (interval, robot->configSize(), robot->numberDof(), constraints),
            parameterSize_ (robot->numberDof()),
            robot_ (robot),
            base_ (robot->RnxSOnConfigSpace()->vectorSpacesMerged()),
            parameters_ ((int)NbCoeffs, parameterSize_),
            velocity_ (parameterSize_)
          {
            powersOfT_(0) = 1;
            for (size_type i = 1; i < NbPowerOfT; ++i)
              powersOfT_(i) = powersOfT_(i - 1) * length();
          }

          Spline (const Spline& path);

          Spline (const Spline& path, const ConstraintSetPtr_t& constraints);

          void init (const Ptr_t& self) { Path::init(self); weak_ = self; }

          std::ostream& print (std::ostream &os) const;

          bool impl_compute (ConfigurationOut_t configuration, value_type t) const;

          void impl_derivative (vectorOut_t res, const value_type& t, size_type order) const;

          void impl_paramDerivative (vectorOut_t res, const value_type& t) const;

          void impl_paramIntegrate (vectorIn_t dParam);

          void impl_velocityBound (vectorOut_t result, const value_type& t0, const value_type& t1) const;

          /// Robot number of degrees of freedom.
          size_type parameterSize_;
          DevicePtr_t robot_;
          /// Base of the spline path.
          /// The spline is a curve in the tangent space of the robot at this
          /// configuration.
          LiegroupElement base_;
          /// Parameters of the spline are stored in a matrix
          ///   number of rows = degree of polynomial + 1
          ///   number of columns = robot number of degrees of freedom.
          ParameterMatrix_t parameters_;

        private:
          WkPtr_t weak_;

          mutable vector_t velocity_;
          mutable PowersOfT_t powersOfT_;

          friend class steeringMethod::Spline<_PolynomeBasis, _Order>;
      }; // class Spline
      /// \}
    } //   namespace path
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_SPLINE_HH
