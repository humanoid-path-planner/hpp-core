// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/core/interpolated-path.hh>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    InterpolatedPath::InterpolatedPath (const DevicePtr_t& device,
        ConfigurationIn_t init,
        ConfigurationIn_t end,
        interval_t timeRange) :
      parent_t (timeRange, device->configSize (),
		device->numberDof ()),
      device_ (device)
    {
      assert (init.size() == device_->configSize ());
      insert (timeRange.first, init);
      insert (timeRange.second, end);
      assert (device);
      assert (length() >= 0);
      assert (!constraints ());
      assert(timeRange.second > timeRange.first || init == end);
    }

    InterpolatedPath::InterpolatedPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
                                interval_t timeRange,
				ConstraintSetPtr_t constraints) :
      parent_t (timeRange, device->configSize (),
		device->numberDof (), constraints),
      device_ (device)
    {
      assert (init.size() == device_->configSize ());
      insert (timeRange.first, init);
      insert (timeRange.second, end);
      assert (device);
      assert (length() >= 0);
      assert(timeRange.second > timeRange.first || init == end);
    }

    InterpolatedPath::InterpolatedPath (const PathPtr_t& path,
                                        const DevicePtr_t& device,
                                        const std::size_t& nbSamples) :
      parent_t (path->timeRange(), device->configSize (),
		device->numberDof (), path->constraints ()), device_ (device)

    {
      assert (path->initial ().size() == device_->configSize ());
      insert (timeRange().first, path->initial ());
      insert (timeRange().second, path->end ());
      assert (device);
      assert (path->length () >= 0);

      const value_type dl = path->length () / (value_type) (nbSamples + 1);
      Configuration_t q (device->configSize ());
      for (std::size_t iS = 0; iS < nbSamples; ++iS) {
        const value_type u = timeRange().first + dl * (value_type) (iS + 1);
        if (!(*path) (q, u))
          throw projection_error ("could not build InterpolatedPath");
        insert (u, q);
      }

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
      InterpolatedPath* ptr = new InterpolatedPath (path, device, nbSamples);
      InterpolatedPathPtr_t shPtr (ptr);
      ptr->init (shPtr);
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
      parent_t::init (self);
      weak_ = self;
      checkPath ();
    }

    bool InterpolatedPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      assert (param >= paramRange().first);
      if (param == paramRange ().first || paramLength() == 0) {
	result.noalias () = initial();
	return true;
      }
      assert (fabs (configs_.rbegin()->first - paramRange ().second)
          < Eigen::NumTraits<value_type>::dummy_precision());
      assert (param < configs_.rbegin()->first +
              Eigen::NumTraits<value_type>::dummy_precision());
      if (param >= configs_.rbegin()->first) {
	param = configs_.rbegin()->first;
	result.noalias () = end();
	return true;
      }
      InterpolationPoints_t::const_iterator itA = configs_.lower_bound (param);
      assert (itA != configs_.end());
      InterpolationPoints_t::const_iterator itB = itA; --itB;
      const value_type T = itA->first - itB->first;
      const value_type u = (param - itB->first) / T;

      pinocchio::interpolate<hpp::pinocchio::RnxSOnLieGroupMap> (device_, itB->second, itA->second, u, result);
      return true;
    }

    void InterpolatedPath::impl_derivative
    (vectorOut_t result, const value_type& s, size_type order) const
    {
      value_type param (s);
      assert (param >= paramRange().first);
      if (paramRange ().first == paramRange ().second) {
	result.setZero ();
	return;
      }
      InterpolationPoints_t::const_iterator itA;
      InterpolationPoints_t::const_iterator itB;
      assert (fabs (configs_.rbegin()->first - paramRange ().second)
          < Eigen::NumTraits<value_type>::dummy_precision());
      if (param >= configs_.rbegin()->first) {
	param = configs_.rbegin()->first;
        itA = configs_.end(); --itA;
        itB = itA; --itB;
      } else {
        itA = configs_.upper_bound (param);
        itB = configs_.lower_bound (param);
        if (itB == itA) {
          if (itB == configs_.begin ()) {
            ++itA;
          } else {
            --itB;
          }
        }
      }
      assert (itA != configs_.end ());
      const value_type T = itA->first - itB->first;
      if (order > 1) {
	result.setZero ();
	return;
      }
      if (order == 1) {
	pinocchio::difference <hpp::pinocchio::RnxSOnLieGroupMap>
	  (device_, itA->second, itB->second, result);
	result = (1/T) * result;
      }
    }

    void InterpolatedPath::impl_velocityBound (vectorOut_t result,
        const value_type& t0, const value_type& t1) const
    {
      InterpolationPoints_t::const_iterator next = configs_.lower_bound (t0);
      InterpolationPoints_t::const_iterator current = next; ++next;

      result.setZero();
      vector_t tmp (result.size());
      while (t1 > current->first) {
	pinocchio::difference <hpp::pinocchio::RnxSOnLieGroupMap>
	  (device_, next->second, current->second, tmp);
        const value_type T = next->first - current->first;
        result.noalias() = result.cwiseMax(tmp.cwiseAbs() / T);
        ++current; ++next;
      }
    }

    PathPtr_t InterpolatedPath::impl_extract (const interval_t& subInterval) const
    {
      // Length is assumed to be proportional to interval range
      const bool reverse = (subInterval.first > subInterval.second);
      const value_type tmin = (reverse)?subInterval.second:subInterval.first ;
      const value_type tmax = (reverse)?subInterval.first :subInterval.second;
      const value_type l = tmax - tmin;

      bool success;
      Configuration_t q1 (configAtParam (subInterval.first, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in InterpolatedPath::extract");
      Configuration_t q2 (configAtParam (subInterval.second, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in InterpolatedPath::extract");
      // TODO if the constraint as a varying right hand side, then the generated
      // path will not be correct.
      // In order to propagate the correct time information, one must either:
      // - implement some extraction method for the constraints.
      // - make class Path tolerate reversed time range, i.e. (t1, t2) with t1 > t2.
      // The second option shouldn't be hard to implement but may be hard to
      // propagate in all the code.
      InterpolatedPathPtr_t result = InterpolatedPath::create (device_, q1, q2,
          l, constraints ());

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
      const value_type& l = paramLength();

      InterpolatedPathPtr_t result =
        InterpolatedPath::create (device_, end(), initial(), l, constraints ());

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

