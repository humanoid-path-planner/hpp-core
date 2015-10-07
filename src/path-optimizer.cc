// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/core/path-optimizer.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    PathPtr_t PathOptimizer::steer (ConfigurationIn_t q1,
        ConfigurationIn_t q2) const
    {
      PathPtr_t dp = (*problem().steeringMethod())(q1,q2);
      if (dp) {
        if (!problem().pathProjector()) return dp;
        PathPtr_t pp;
        if (problem().pathProjector()->apply (dp, pp))
          return pp;
      }
      return PathPtr_t ();
    }
  } // namespace core
} // namespace hpp

