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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    StraightPath::StraightPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type length) :
      parent_t (interval_t (0, length), device->configSize (),
		device->numberDof ()),
      device_ (device), initial_ (init), end_ (end)
    {
      assert (device);
      assert (length >= 0);
      assert (!constraints ());
    }

    StraightPath::StraightPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type length,
				ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, length), device->configSize (),
		device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end)
    {
      assert (device);
      assert (length >= 0);
    }

    StraightPath::StraightPath (const StraightPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_)
    {
    }

    StraightPath::StraightPath (const StraightPath& path,
				const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }

    bool StraightPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      if (param == timeRange ().first || timeRange ().second == 0) {
	result = initial_;
	return true;
      }
      if (param == timeRange ().second) {
	result = end_;
	return true;
      }
      value_type u = param/timeRange ().second;
      if (timeRange ().second == 0)
	u = 0;
      model::interpolate (device_, initial_, end_, u, result);
      return true;
    }

    PathPtr_t StraightPath::extract (const interval_t& subInterval) const
      throw (projection_error)
    {
      // Length is assumed to be proportional to interval range
      value_type l = fabs (subInterval.second - subInterval.first);

      bool success;
      Configuration_t q1 ((*this) (subInterval.first, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in StraightPath::extract");
      Configuration_t q2 ((*this) (subInterval.second, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in StraightPath::extract");
      PathPtr_t result = StraightPath::create (device_, q1, q2, l,
					       constraints ());
      return result;
    }

    DevicePtr_t StraightPath::device () const
    {
      return device_;
    }
  } //   namespace core
} // namespace hpp

