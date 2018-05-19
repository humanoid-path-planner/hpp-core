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

# include <sstream>

# include <pinocchio/algorithm/joint-configuration.hpp>

# include <hpp/pinocchio/device.hh>

# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    /// \addtogroup configuration_sampling
    /// \{

    /// Uniformly sample with bounds of degrees of freedom.
    class HPP_CORE_DLLAPI BasicConfigurationShooter :
      public ConfigurationShooter
    {
    public:
      static BasicConfigurationShooterPtr_t create (const DevicePtr_t& robot)
      {
	BasicConfigurationShooter* ptr = new BasicConfigurationShooter (robot);
	BasicConfigurationShooterPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      virtual ConfigurationPtr_t shoot () const;

    protected:
      /// Uniformly sample configuration space
      ///
      /// Note that translation joints have to be bounded.
      BasicConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
      {
      }
      void init (const BasicConfigurationShooterPtr_t& self)
      {
	ConfigurationShooter::init (self);
	weak_ = self;
      }

    private:
      DevicePtr_t robot_;
      BasicConfigurationShooterWkPtr_t weak_;
    }; // class BasicConfigurationShooter
    /// \}
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH
