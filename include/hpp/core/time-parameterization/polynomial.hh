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

#ifndef HPP_CORE_TIME_PARAMETERIZATION_POLYNOMIAL_HH
# define HPP_CORE_TIME_PARAMETERIZATION_POLYNOMIAL_HH

# include <hpp/constraints/differentiable-function.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/time-parameterization.hh>
# include <hpp/core/path/math.hh>

namespace hpp {
  namespace core {
    namespace timeParameterization {
      class HPP_CORE_DLLAPI Polynomial : public TimeParameterization
      {
        public:
          Polynomial (const vector_t& param) : a (param)
          {
            for (size_type i=0; i<a.size(); ++i){
              assert(a[i] < std::numeric_limits<value_type>::infinity());
              assert(a[i] > -std::numeric_limits<value_type>::infinity());
            }
          }

          const vector_t& parameters () const
          {
            return a;
          }

          TimeParameterizationPtr_t copy () const
          {
            return TimeParameterizationPtr_t (new Polynomial (*this));
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

          /// Compute the bound of the derivative on \f$ [ low, up ] \f$.
          /// Three cases are handled:
          /// \li first order: \f$ B = |a_1| \f$
          /// \li second order: \f$ B = \max{|J(low)|, |J(up)|} \f$
          /// \li third order:
          ///     Let \f$ x_m = - \frac{a_2}{3 a_3} \f$ be the extremal point
          ///     of the derivative and \f$ M = \max{|J(low)|, |J(up)|}\f$.
          ///     Then:
          ///     - if \f$ low < x_m < up \f$,
          ///       \f$ B = \max{ |a_1 - \frac{a_2}{3 a_3}|, M } \f$
          ///     - else \f$ B = M \f$
          value_type derivativeBound (const value_type& low, const value_type& up) const
          {
            using std::max;
            using std::fabs;
            switch (a.size()) {
              case 2:
                return fabs(a[1]);
                break;
              case 3:
                return max (fabs(Jac(low)), fabs(Jac(up)));
                break;
              case 4:
                {
                  const value_type x_m = - a[2] / (3 * a[3]);
                  const value_type M = max(fabs(Jac(low)), fabs(Jac(up)));
                  if (low < x_m && x_m < up)
                    return max (M, fabs(a[1] - a[2] / 3*a[3]));
                  else
                    return M;
                }
                break;
              default:
                throw std::logic_error("not implemented");
            }
          }

        private:
          value_type val (const value_type& t) const
          {
            value_type tn = 1;
            value_type res = a[0];
            for (size_type i = 1; i < a.size(); ++i)
            {
              tn *= t;
              res += a[i] * tn;
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
            if (order >= a.size()) return 0;
            const size_type MaxOrder = 10;
            if (a.size() > MaxOrder)
              throw std::invalid_argument ("Cannot compute the derivative of order greater than 10.");
            typedef path::binomials<MaxOrder> Binomials_t;
            const Binomials_t::Factorials_t& factors = Binomials_t::factorials();

            value_type res = 0;
            value_type tn = 1;
            for (size_type i = order; i < a.size(); ++i)
            {
              res += value_type(factors[i]/factors[i-order]) * a[i] * tn;
              tn *= t;
            }
            return res;
          }

          vector_t a;
      }; // class Polynomial
    } // namespace timeParameterization
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_TIME_PARAMETERIZATION_POLYNOMIAL_HH
