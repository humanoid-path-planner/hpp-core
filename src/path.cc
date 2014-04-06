//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include "extracted-path.hh"

namespace hpp {
  namespace core {
    void Path::impl_derivative (gradient_t& derivative,
				value_type argument,
				size_type order) const throw ()
    {
      if (order == 0) {
	impl_compute (derivative, argument);
      } else {
	throw std::runtime_error ("Not implemented");
      }
    }

    void Path::impl_derivative (gradient_t& g, StableTimePoint_t p,
				size_type order) const throw ()
    {
      value_type t = p.getTime (timeRange_);
      return impl_derivative (g, t, order);
    }

    PathPtr_t Path::extract (const interval_t& subInterval) const
    {
      if (subInterval == timeRange_)
	return this->copy ();
      return ExtractedPath::create (weak_.lock (), subInterval);
    }

  } //   namespace core
} // namespace hpp
