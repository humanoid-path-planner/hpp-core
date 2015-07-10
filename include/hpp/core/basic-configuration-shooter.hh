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
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
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
      virtual ConfigurationPtr_t shoot () const
      {
	JointVector_t jv = robot_->getJointVector ();
	ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); itJoint++) {
	  std::size_t rank = (*itJoint)->rankInConfiguration ();
	  (*itJoint)->configuration ()->uniformlySample (rank, *config);
	}
	// Shoot extra configuration variables
	size_type extraDim = robot_->extraConfigSpace ().dimension ();
	size_type offset = robot_->configSize () - extraDim;
	for (size_type i=0; i<extraDim; ++i) {
	  value_type lower = robot_->extraConfigSpace ().lower (i);
	  value_type upper = robot_->extraConfigSpace ().upper (i);
	  value_type range = upper - lower;
	  if ((range < 0) ||
	      (range == std::numeric_limits<double>::infinity())) {
	    std::ostringstream oss
	      ("Cannot uniformy sample extra config variable ");
	    oss << i << ". min = " <<lower<< ", max = " << upper << std::endl;
	    throw std::runtime_error (oss.str ());
	  }
	  (*config) [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
	}
	return config;
      }
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
      const DevicePtr_t& robot_;
      BasicConfigurationShooterWkPtr_t weak_;
    }; // class BasicConfigurationShooter
    /// \}
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH
