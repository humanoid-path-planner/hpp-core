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

#ifndef HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH
# define HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH

# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    class BasicConfigurationShooter : public ConfigurationShooter
    {
    public:
      BasicConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
      {
      }
      virtual ConfigurationPtr_t shoot () const
      {
	JointVector_t jv = robot_->getJointVector ();
	ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); itJoint++) {
	  std::size_t rank = (*itJoint)->rankInConfiguration ();
	  (*itJoint)->configuration ()->uniformlySample (rank, *config);
	}
	return config;
      }
    private:
      const DevicePtr_t& robot_;
    }; // class BasicConfigurationShooter
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH
