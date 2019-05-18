//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH
# define HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH

# include <sstream>

# include <hpp/pinocchio/device.hh>

# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {
      /// \addtogroup configuration_sampling
      /// \{

      /// Sample configuration using a gaussian distribution around a
      /// configuration.
      class HPP_CORE_DLLAPI Gaussian :
        public ConfigurationShooter
      {
        public:
          static GaussianPtr_t create (const DevicePtr_t& robot)
          {
            Gaussian* ptr = new Gaussian (robot);
            GaussianPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }
          virtual void shoot (Configuration_t& q) const;

          void center (ConfigurationIn_t c)
          {
            center_ = c;
          }
          const Configuration_t& center () const
          {
            return center_;
          }

          /// Set the standard deviation proportional to a default value
          ///
          /// The default value is:
          /// \li for vector spaces, the difference between the upper and the
          ///     lower bounds,
          /// \li for SO(n), \f$\frac{2\pi}{\sqrt{2n-3}}\f$ on each of the
          ///     \f$ 2n-3 \f$ dimensions,
          /// \li SE(n) is treated as \f$ R^n \times SO(n) \f$.
          void sigma (const value_type& factor);

          void sigmas (vectorIn_t s)
          {
            assert (s.size() == robot_->numberDof());
            sigmas_ = s;
          }
          const vector_t& sigmas () const
          {
            return sigmas_;
          }

        protected:
          /// Create a gaussian distribution centered in the robot current
          /// configuration. The standard deviation is computed as \c sigma(0.25)
          /// \sa Gaussian::sigma
          Gaussian (const DevicePtr_t& robot)
            : robot_ (robot)
            , center_ (robot->currentConfiguration())
            , sigmas_ (robot->numberDof())
          {
            sigma(1./4.);
          }
          void init (const GaussianPtr_t& self)
          {
            ConfigurationShooter::init (self);
            weak_ = self;
          }

        private:
          const DevicePtr_t& robot_;
          /// The mean value
          Configuration_t center_;
          /// The standard deviation
          vector_t sigmas_;

          GaussianWkPtr_t weak_;
      }; // class Gaussian
      /// \}
    } //   namespace configurationShooter
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_CONFIGURATION_SHOOTER_GAUSSIAN_HH
