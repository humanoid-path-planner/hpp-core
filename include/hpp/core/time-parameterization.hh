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

#ifndef HPP_CORE_TIME_PARAMETERIZATION_HH
# define HPP_CORE_TIME_PARAMETERIZATION_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

# include <hpp/constraints/differentiable-function.hh>

namespace hpp {
  namespace core {
    class HPP_CORE_DLLAPI TimeParameterization :
      public DifferentiableFunction
    {
      public:
        value_type value (const value_type& t) const
        {
          return this->operator() (map11_t(&t)).vector()[0];
        }
        value_type derivative (const value_type& t) const
        {
          matrix11_t der;
          jacobian(der, map11_t(&t));
          return der[0];
        };
        value_type derivativeBound (const value_type& low, const value_type& up) const
        {
          assert (low <= up);
          return impl_derivativeBound (low, up);
        }

      protected:
        TimeParameterization (std::string name) : DifferentiableFunction (1, 1, 1, name) {}

        virtual value_type impl_derivativeBound (const value_type&, const value_type&) const
        {
          throw std::logic_error("not implemented");
        }

      private:
        typedef Eigen::Matrix<value_type, 1, 1> matrix11_t;
        typedef Eigen::Map<const matrix11_t> map11_t;
    }; // class TimeParameterization
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_TIME_PARAMETERIZATION_HH
