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

#ifndef HPP_CORE_PATH_MATH_HH
#define HPP_CORE_PATH_MATH_HH

namespace hpp {
  namespace core {
    namespace path {
      template <int N> struct binomials {
        typedef Eigen::Matrix<size_type, N, 1> Factorials_t;

        static inline const Factorials_t& factorials ()
        {
          static Factorials_t ret (Factorials_t::Zero());
          if (ret(0)==0) {
            ret (0) = 1;
            for (size_type i = 1; i < N; ++i) ret(i) = ret(i-1) * i;
          }
          return ret;
        }

        static inline size_type binomial (size_type n, size_type k)
        {
          const Factorials_t& factors = factorials();
          assert (n >= k && k >= 0);
          assert (n < N);
          size_type denom = factors(k) * factors(n - k);
          return factors (n) / denom;
        }
      };
    } //   namespace path
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_MATH_HH
