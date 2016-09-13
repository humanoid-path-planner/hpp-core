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
#include <hpp/util/timer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    namespace {
      HPP_DEFINE_TIMECOUNTER (PathProjection);
    }

    PathProjector::PathProjector (const DistancePtr_t& distance,
				  const SteeringMethodPtr_t& steeringMethod,
				  bool keepSteeringMethodConstraints) :
      distance_ (distance), steeringMethod_ (steeringMethod->copy ())
    {
      assert (distance_ != NULL);
      assert (steeringMethod_ != NULL);
      if (!keepSteeringMethodConstraints) {
	steeringMethod_->constraints (ConstraintSetPtr_t ());
      }
    }

    PathProjector::~PathProjector ()
    {
      HPP_DISPLAY_TIMECOUNTER (PathProjection);
      HPP_RESET_TIMECOUNTER (PathProjection);
    }

    value_type PathProjector::d (ConfigurationIn_t q1, ConfigurationIn_t q2) const
    {
      return (*distance_) (q1, q2);
    }

    PathPtr_t PathProjector::steer (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const
    {
      PathPtr_t result ((*steeringMethod_) (q1, q2));
      // In the case of hermite path, we want the paths to be constrained.
      // assert (!result->constraints ());
      return result;
    }

    bool PathProjector::apply (const PathPtr_t& path,
			       PathPtr_t& proj) const
    {
      HPP_START_TIMECOUNTER (PathProjection);
      bool ret = impl_apply (path, proj);
      HPP_STOP_TIMECOUNTER (PathProjection);
      return ret;
    }
  } // namespace core
} // namespace hpp
