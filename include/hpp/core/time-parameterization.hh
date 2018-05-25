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

namespace hpp {
  namespace core {
    class HPP_CORE_DLLAPI TimeParameterization
    {
      public:
        virtual value_type value (const value_type& t) const = 0;
        virtual value_type derivative (const value_type& t, const size_type& order) const = 0;
        virtual value_type derivativeBound (const value_type& low, const value_type& up) const
        {
          (void) low;
          (void) up;
          throw std::logic_error("not implemented");
        }

        virtual TimeParameterizationPtr_t copy () const = 0;
    }; // class TimeParameterization
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_TIME_PARAMETERIZATION_HH
