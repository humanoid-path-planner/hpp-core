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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH
# define HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH

# include <sstream>

# include <hpp/pinocchio/device.hh>

# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {
      /// \addtogroup configuration_sampling
      /// \{

      /// Uniformly sample with bounds of degrees of freedom.
      class HPP_CORE_DLLAPI Uniform : public ConfigurationShooter
      {
      public:
        static UniformPtr_t create (const DevicePtr_t& robot)
        {
          Uniform* ptr = new Uniform (robot);
          UniformPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }
        virtual void shoot (Configuration_t& q) const;

        void sampleExtraDOF(bool sampleExtraDOF){
            sampleExtraDOF_=sampleExtraDOF;
        }

      protected:
        /// Uniformly sample configuration space
        ///
        /// Note that translation joints have to be bounded.
        Uniform (const DevicePtr_t& robot) : robot_ (robot),sampleExtraDOF_(true)
        {
        }
        void init (const UniformPtr_t& self)
        {
          ConfigurationShooter::init (self);
          weak_ = self;
        }

      private:
        DevicePtr_t robot_;
        bool sampleExtraDOF_;
        UniformWkPtr_t weak_;
      }; // class Uniform
      /// \}
    } // namespace configurationShooter
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH
