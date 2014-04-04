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
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/straight-path.hh>

namespace hpp {
  namespace core {
    StraightPath::StraightPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type length) :
      parent_t (interval_t (0, length), device->configSize ()),
      device_ (device), initial_ (init), end_ (end)
    {
      assert (length >= 0);
    }

    StraightPath::StraightPath (const StraightPath& path) :
      parent_t (path), initial_ (path.initial_), end_ (path.end_)
    {
    }

    void StraightPath::impl_compute (result_t& result, value_type param) const
      throw ()
    {
      if (param == timeRange ().first || timeRange ().second == 0) {
	result = initial_;
	return;
      }
      if (param == timeRange ().second) {
	result = end_;
	return;
      }
      value_type u = param/timeRange ().second;
      if (timeRange ().second == 0)
	u = 0;
      // Loop over device joint and interpolate
      const JointVector_t& jv (device_->getJointVector ());
      for (model::JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); itJoint++) {
	std::size_t rank = (*itJoint)->rankInConfiguration ();
	(*itJoint)->configuration ()->interpolate
	  (initial_, end_, u, rank, result);
      }
    }

  } //   namespace core
} // namespace hpp

