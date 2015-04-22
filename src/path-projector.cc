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

#include "hpp/core/path-projector.hh"

#include <hpp/util/pointer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/distance.hh>

namespace hpp {
  namespace core {
    PathProjector::PathProjector (const core::DistancePtr_t distance) :
      distance_ (distance)
    {}

    PathProjector::~PathProjector ()
    {}

    value_type PathProjector::d (ConfigurationIn_t q1, ConfigurationIn_t q2) const
    {
      assert (distance_ != NULL);
      return (*distance_) (q1, q2);
    }

    bool PathProjector::apply (const PathPtr_t& path,
			       PathPtr_t& proj) const
    {
      return impl_apply (path, proj);
    }
  } // namespace core
} // namespace hpp
