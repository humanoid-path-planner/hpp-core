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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    InterpolatedPath::InterpolatedPath (const DevicePtr_t& device,
        ConfigurationIn_t init,
        ConfigurationIn_t end,
        value_type length) :
      parent_t (interval_t (0, length), device->configSize (),
		device->numberDof ()),
      device_ (device)
    {
      assert (init.size() == device_->configSize ());
      insert (0, init);
      insert (length, end);
      assert (device);
      assert (length >= 0);
      assert (!constraints ());
    }

    InterpolatedPath::InterpolatedPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type length,
				ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, length), device->configSize (),
		device->numberDof (), constraints),
      device_ (device)
    {
      assert (init.size() == device_->configSize ());
      insert (0, init);
      insert (length, end);
      assert (device);
      assert (length >= 0);
    }

    InterpolatedPath::InterpolatedPath (const InterpolatedPath& path) :
      parent_t (path), device_ (path.device_), configs_ (path.configs_)
    {
      assert (initial().size() == device_->configSize ());
    }

    InterpolatedPath::InterpolatedPath (const InterpolatedPath& path,
				const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      configs_ (path.configs_)
    {
    }

    InterpolatedPathPtr_t InterpolatedPath::create (const PathPtr_t& path,
        const DevicePtr_t& device, const std::size_t& nbSamples)
    {
      // TODO: If it is a path vector, should we get the waypoints and build
      //       a interpolated path from those waypoints ?
      InterpolatedPath* ptr = new InterpolatedPath (device,
          path->initial (), path->end(), path->length(),
          path->constraints ());
      InterpolatedPathPtr_t shPtr (ptr);
      ptr->init (shPtr);

      const value_type dl = path->length () / (nbSamples + 1);
      Configuration_t q (device->configSize ());
      for (std::size_t iS = 0; iS < nbSamples; ++iS) {
        const value_type u = dl * (iS + 1);
        if (!(*path) (q, u))
          throw projection_error ("could not build InterpolatedPath");
        ptr->insert (u, q);
      }

      return shPtr;
    }

    void InterpolatedPath::init (InterpolatedPathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
      checkPath ();
    }

    void InterpolatedPath::initCopy (InterpolatedPathPtr_t self)
    {
      parent_t::initCopy (self);
      weak_ = self;
      checkPath ();
    }

    bool InterpolatedPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      assert (param >= timeRange().first);
      if (param == timeRange ().first || timeRange ().second == 0) {
	result.noalias () = initial();
	return true;
      }
      if (param >= timeRange ().second) {
	result.noalias () = end();
	return true;
      }
      InterpolationPoints_t::const_iterator itA = configs_.lower_bound (param);
      InterpolationPoints_t::const_iterator itB = itA; --itB;
      const value_type T = itA->first - itB->first;
      const value_type u = (param - itB->first) / T;

      model::interpolate (device_, itB->second, itA->second, u, result);
      return true;
    }

    PathPtr_t InterpolatedPath::extract (const interval_t& subInterval) const
        throw (projection_error)
    {
      // Length is assumed to be proportional to interval range
      const bool reverse = (subInterval.first > subInterval.second);
      const value_type tmin = (reverse)?subInterval.second:subInterval.first ;
      const value_type tmax = (reverse)?subInterval.first :subInterval.second;
      const value_type l = tmax - tmin;

      bool success;
      Configuration_t q1 ((*this) (subInterval.first, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in InterpolatedPath::extract");
      Configuration_t q2 ((*this) (subInterval.second, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in InterpolatedPath::extract");
      InterpolatedPathPtr_t result = InterpolatedPath::create (device_, q1, q2, l,
					       constraints ());

      InterpolationPoints_t::const_iterator it = configs_.upper_bound (tmin);
      if (reverse)
        for (; it->first < tmax; ++it)
          result->insert (l - (it->first - tmin), it->second); 
      else
        for (; it->first < tmax; ++it)
          result->insert (it->first - tmin, it->second); 

      return result;
    }

    PathPtr_t InterpolatedPath::reverse () const
    {
      const value_type& l = length();

      InterpolatedPathPtr_t result =
        InterpolatedPath::create (device_, end(), initial(), length(),
					       constraints ());

      if (configs_.size () > 2) {
        InterpolationPoints_t::const_reverse_iterator it = configs_.rbegin();
        ++it;
        InterpolationPoints_t::const_reverse_iterator itEnd = configs_.rend();
        --itEnd;
        for (; it != itEnd; ++it)
          result->insert (l - it->first, it->second); 
      }

      return result;
    }

    DevicePtr_t InterpolatedPath::device () const
    {
      return device_;
    }
  } //   namespace core
} // namespace hpp

