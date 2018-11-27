//
// Copyright (c) 2018 CNRS
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

#include <iterator>
#include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      inline const value_type& first  (const interval_t& I, bool reverse)
      {
        return reverse ? I.second : I.first;
      }
      inline const value_type& second (const interval_t& I, bool reverse)
      {
        return first(I, !reverse);
      }

      inline const interval_t& begin (const std::list<interval_t>& I, bool reverse)
      {
        return reverse ? I.back() : I.front();
      }
      inline const interval_t& end   (const std::list<interval_t>& I, bool reverse)
      {
        return begin(I, !reverse);
      }
      inline const interval_t& Nth (const std::list<interval_t>& I, int N, bool reverse)
      {
        if (reverse) {
          std::list<interval_t>::const_reverse_iterator it (I.rbegin());
          std::advance(it, N);
          return *it;
        } else {
          std::list<interval_t>::const_iterator         it (I. begin());
          std::advance(it, N);
          return *it;
        }
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
