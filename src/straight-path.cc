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

#include <hpp/core/straight-path.hh>

#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception.hh>
#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/util.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    StraightPathPtr_t StraightPath::create (const DevicePtr_t& device,
        ConfigurationIn_t init,
        ConfigurationIn_t end,
        interval_t interval,
        ConstraintSetPtr_t constraints)
    {
      return create(device->RnxSOnConfigSpace(), init, end, interval, constraints);
    }

    StraightPath::StraightPath (LiegroupSpacePtr_t space,
                                vectorIn_t init,
                                vectorIn_t end,
				interval_t interval) :
      parent_t (interval, space->nq(), space->nv()),
      space_ (space),
      initial_ (init), end_ (end)
    {
      assert (interval.second >= interval.first);
      assert (!constraints ());
    }

    StraightPath::StraightPath (LiegroupSpacePtr_t space,
				vectorIn_t init,
				vectorIn_t end,
				interval_t interval,
                                ConstraintSetPtr_t constraints) :
      parent_t (interval, space->nq(), space->nv(), constraints),
      space_ (space),
      initial_ (init), end_ (end)
    {
      assert (interval.second >= interval.first);
    }

    StraightPath::StraightPath (const StraightPath& path) :
      parent_t (path), space_ (path.space_),
      initial_ (path.initial_),
      end_ (path.end_)
    {
    }

    StraightPath::StraightPath (const StraightPath& path,
				const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), space_ (path.space_),
      initial_ (path.initial_), end_ (path.end_)
    {
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }

    bool StraightPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      const value_type L = paramLength();
      if (param == paramRange ().first || L == 0) {
	result = initial_;
	return true;
      }
      if (param == paramRange ().second) {
	result = end_;
	return true;
      }
      value_type u = (param - paramRange().first) / L;
      if (L == 0) u = 0;
      space_->interpolate(initial_, end_, u, result);
      return true;
    }

    void StraightPath::impl_derivative (vectorOut_t result, const value_type&,
					size_type order) const
    {
      if (order > 1) {
	result.setZero ();
	return;
      }
      if (order == 1) {
	if (paramRange ().first == paramRange ().second) {
	  result.setZero ();
	  return;
	}
        result = space_->elementConstRef(end_) - space_->elementConstRef(initial_);
        result /= paramLength();
	return;
      }
      std::ostringstream oss;
      oss << "order of derivative (" << order << ") should be positive.";
      HPP_THROW_EXCEPTION (hpp::Exception, oss.str ());
    }

    void StraightPath::impl_velocityBound (
        vectorOut_t result, const value_type&, const value_type&) const
    {
      if (paramRange ().first == paramRange ().second) {
        result.setZero();
        return;
      }
      result = space_->elementConstRef(end_) - space_->elementConstRef(initial_);
      result.noalias() = result.cwiseAbs() / paramLength();
    }

    PathPtr_t StraightPath::impl_extract (const interval_t& subInterval) const
    {
      // Length is assumed to be proportional to interval range
      value_type l = fabs (subInterval.second - subInterval.first);

      bool success;
      Configuration_t q1 (configAtParam (subInterval.first, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in StraightPath::extract");
      Configuration_t q2 (configAtParam (subInterval.second, success));
      if (!success) throw projection_error
		      ("Failed to apply constraints in StraightPath::extract");
      StraightPathPtr_t result = StraightPath::create (space_, q1, q2, interval_t(0,l), constraints ());
      return result;
    }

    std::ostream& StraightPath::print (std::ostream &os) const
    {
      Path::print (os << "StraightPath:") << incendl
        << "initial configuration: " << one_line(initial_) << iendl
        << "final configuration:   " << one_line(end_) << decendl;
      return os;
    }

    template<class Archive>
    void StraightPath::serialize(Archive & ar, const unsigned int version)
    {
      using namespace boost::serialization;
      (void) version;
      ar & make_nvp("base", base_object<Path>(*this));
      ar & BOOST_SERIALIZATION_NVP(space_);
      ar & BOOST_SERIALIZATION_NVP(initial_);
      ar & BOOST_SERIALIZATION_NVP(end_);
      ar & BOOST_SERIALIZATION_NVP(weak_);
    }

    HPP_SERIALIZATION_IMPLEMENT(StraightPath);
  } //   namespace core
} // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::core::StraightPath)
