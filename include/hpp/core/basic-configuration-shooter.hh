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

# warning "This file is deprecated. You should only include hpp/core/configuration-shooter/uniform.hh"

# include <hpp/core/configuration-shooter/uniform.hh>

namespace hpp {
  namespace core {
    /// \addtogroup configuration_sampling
    /// \{
    typedef configurationShooter::Uniform      BasicConfigurationShooter;
    typedef configurationShooter::UniformPtr_t BasicConfigurationShooterPtr_t;
    /// \}
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_BASIC_CONFIGURATION_SHOOTER_HH
