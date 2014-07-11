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

#ifndef HPP_CORE_COLLISION_VALIDATION_HH
# define HPP_CORE_COLLISION_VALIDATION_HH

# include <hpp/core/config-validation.hh>

namespace hpp {
  namespace core {
    /// Validate a configuration with respect to collision
    ///
    class HPP_CORE_DLLAPI CollisionValidation : public ConfigValidation
    {
    public:
      static CollisionValidationPtr_t create (const DevicePtr_t& robot);
      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \param report if true throw an exception if config is invalid.
      /// \return whether the whole config is valid.
      virtual bool validate (const Configuration_t& config,
			     bool report = false);
    protected:
      CollisionValidation (const DevicePtr_t& robot);
    private:
      DevicePtr_t robot_;
    }; // class ConfigValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COLLISION_VALIDATION_HH
