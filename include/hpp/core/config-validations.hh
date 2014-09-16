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

#ifndef HPP_CORE_CONFIG_VALIDATIONS_HH
# define HPP_CORE_CONFIG_VALIDATIONS_HH

# include <hpp/core/config-validation.hh>

namespace hpp {
  namespace core {
    /// Validate a configuration with respect to collision
    ///
    class HPP_CORE_DLLAPI ConfigValidations : public ConfigValidation
    {
    public:
      static ConfigValidationsPtr_t create ();
      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \param report if true throw an exception if config is invalid.
      /// \return whether the whole config is valid.
      virtual bool validate (const Configuration_t& config,
			     bool report = false);
      /// Add a configuration validation object
      void add (const ConfigValidationPtr_t& configValidation);

      /// Add an obstacle
      /// \param object obstacle added
      /// Store obstacle and build a collision pair with each body of the robot.
      virtual void addObstacle (const CollisionObjectPtr_t& object);
    protected:
      ConfigValidations ();
    private:
      std::vector <ConfigValidationPtr_t> validations_;
    }; // class ConfigValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_CONFIG_VALIDATIONS_HH
