// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//          Olivier Roussel (olivier.roussel@laas.fr)
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

#ifndef HPP_CORE_TIME_PARAMETERIZATION_PIECEWISE_POLYNOMIAL_HH
# define HPP_CORE_TIME_PARAMETERIZATION_PIECEWISE_POLYNOMIAL_HH

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/core/fwd.hh>
#include <hpp/core/config.hh>
#include <hpp/core/time-parameterization.hh>
#include <hpp/core/path/math.hh>

namespace hpp {
  namespace core {
    namespace timeParameterization {

      template <int _Order>
      class HPP_CORE_DLLAPI PiecewisePolynomial : public TimeParameterization
      {
        public:
          enum {
            Order = _Order,
            NbCoeffs = Order + 1,
          };

          typedef Eigen::Matrix<value_type, NbCoeffs, Eigen::Dynamic, Eigen::ColMajor> ParameterMatrix_t;
          typedef Eigen::Matrix<value_type, Eigen::Dynamic, 1> Vector_t;
          
          /** Construct piecewise polynomials of order k and defined by N polynomial.
           * \param parameters Matrix of polynomials coefficients (size k x N).
           * \param breakpoints Domain of the piecewise polynomial, defined by N+1 breakpoints 
           *                    defining the half-open intervals of each polynomial.
           */ 
          PiecewisePolynomial (const ParameterMatrix_t& parameters, 
                               const Vector_t& breakpoints) : 
            parameters_ (parameters),
            breakpoints_(breakpoints)
          {
            assert(breakpoints_.size() == parameters_.cols()+1);
            assert(parameters_.rows() == NbCoeffs);

            for (size_type j=0; j<parameters_.cols(); ++j){
              if (j > 0){
                assert(breakpoints_[j] > breakpoints_[j-1]);
              }
              for (size_type i=0; i<parameters_.rows(); ++i){
                assert(parameters_(i,j) < std::numeric_limits<value_type>::infinity());
                assert(parameters_(i,j) > -std::numeric_limits<value_type>::infinity());
              }
            }

          }

          const ParameterMatrix_t& parameters () const
          {
            return parameters_;
          }

          TimeParameterizationPtr_t copy () const
          {
            return TimeParameterizationPtr_t (new PiecewisePolynomial (*this));
          }

          /// Computes \f$ \sum_{i=0}^n a_i t^i \f$
          value_type value (const value_type& t) const
          {
            return val (t);
          }

          /// Computes \f$ \sum_{i=1}^n i a_i t^{i-1} \f$
          value_type derivative (const value_type& t, const size_type& order) const
          {
            return Jac(t, order);
          }

        private:
          value_type val (const value_type& t) const
          {
            const size_t seg_index = findPolynomialIndex(t);
            const auto& poly_coeffs = parameters_.col(seg_index);
            value_type tn = 1;
            value_type res = poly_coeffs[0];
            for (size_type i = 1; i < poly_coeffs.size(); ++i)
            {
              tn *= t;
              res += poly_coeffs[i] * tn;
            }
            assert (res == res);
            return res;
          }

          value_type Jac (const value_type& t) const
          {
            return Jac(t,1);
          }

          value_type Jac (const value_type& t, const size_type& order) const
          {
            if (order >= parameters_.rows()) return 0;
            const size_type MaxOrder = 10;
            if (parameters_.rows() > MaxOrder)
              throw std::invalid_argument ("Cannot compute the derivative of order greater than 10.");
            typedef path::binomials<MaxOrder> Binomials_t;
            const Binomials_t::Factorials_t& factors = Binomials_t::factorials();

            const size_t seg_index = findPolynomialIndex(t);
            const auto& poly_coeffs = parameters_.col(seg_index);
            
            value_type res = 0;
            value_type tn = 1;
            for (size_type i = order; i < poly_coeffs.size(); ++i)
            {
              res += value_type(factors[i]/factors[i-order]) * poly_coeffs[i] * tn;
              tn *= t;
            }
            return res;
          }

          size_t findPolynomialIndex(const value_type& t) const {
            size_t seg_index = std::numeric_limits<size_t>::max();
            for (int i = 0; i < parameters_.size(); ++i) {
              if (breakpoints_[i] <= t && t <= breakpoints_[i + 1]) {
                seg_index = i;
                break;
              }
            }
            if (seg_index == std::numeric_limits<size_t>::max()) {
              std::ostringstream oss;
              oss << "Position " << t << " is outside of range [ " << breakpoints_[0]
                << ", " << breakpoints_[breakpoints_.size()-1] << ']';
              throw std::runtime_error(oss.str());
            }
            return seg_index;
          }

          /// Parameters of the polynomials are stored in a matrix
          ///   number of rows = degree of polynomial + 1
          ///   number of columns = number of polynomials N
          ParameterMatrix_t parameters_;
          Vector_t breakpoints_; // size N + 1
      }; // class PiecewisePolynomial
    } // namespace timeParameterization
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_TIME_PARAMETERIZATION_PIECEWISE_POLYNOMIAL_HH
