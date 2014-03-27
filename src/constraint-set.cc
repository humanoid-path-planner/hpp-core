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

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (ConfigProjectorTrivial);
    typedef boost::shared_ptr <ConfigProjectorTrivial>
    ConfigProjectorTrivialPtr_t;
    class ConfigProjectorTrivial : public ConfigProjector
    {
    public:
      static ConfigProjectorTrivialPtr_t create (const DevicePtr_t& robot)
      {
	ConfigProjectorTrivial* ptr = new ConfigProjectorTrivial (robot);
	ConfigProjectorTrivialPtr_t shPtr (ptr);
	return shPtr;
      }
      ConfigProjectorTrivial (const DevicePtr_t& robot)
	: ConfigProjector (robot, "trivial ConfigProjector", 0, 0)
      {
      }
      bool impl_compute (Configuration_t& configuration)
      {
	computeLockedDofs (configuration);
	return true;
      }
    }; // class ConfigProjectorTrivial

    bool ConstraintSet::impl_compute (Configuration_t& configuration)
    {
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); itConstraint ++) {
	if (!(*itConstraint)->impl_compute (configuration)) return false;
      }
      return true;
    }
    ConstraintSet::ConstraintSet (const DevicePtr_t& robot,
				  const std::string& name) :
      Constraint (name), constraints_ (), configProjector_ (),
      hasLockedDofs_ (false)
    {
      constraints_.push_back (ConfigProjectorTrivial::create (robot));
    }

    void ConstraintSet::removeFirstElement ()
    {
      constraints_.pop_front ();
    }
  } // namespace core
} // namespace core
