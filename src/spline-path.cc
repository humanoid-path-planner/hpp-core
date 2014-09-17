//
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
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

#include <hpp/core/spline-path.hh>

namespace hpp {
  namespace core {

  	void SplinePath::impl_compute (ConfigurationOut_t result,
     value_type t) const
    {
		roboptim::CubicBSpline::result_t tmpResult(outputSize ());
		(*spline_)(tmpResult,t);
		result = Eigen::Ref<Configuration_t> (tmpResult);
    }

    } //   namespace core
} // namespace hpp
