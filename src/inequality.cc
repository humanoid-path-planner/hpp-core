// Copyright (c) 2014, LAAS-CNRS
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

#include "hpp/core/inequality.hh"

namespace hpp {
  namespace core {
    EqualityPtr_t Equality::unique_ = Equality::create ();

    bool InequalityVector::operator () (vector_t& value, matrix_t& jacobian) const
    {
      bool isPassive = true;
      for (size_type i = 0; i < invert_.size (); i++) {
        if (invert_[i] * value[i] <= 0) {
          value[i] = value[i] - invert_[i] * threshold_;
          isPassive = false;
        } else if (invert_[i] * value[i] < threshold_) {
          value[i] -= invert_[i] * threshold_;
        } else {
          value[i] = 0;
          jacobian.row (i).setZero ();
        }
      }
      return !isPassive;
    }
  } // namespace core
} // namespace hpp
