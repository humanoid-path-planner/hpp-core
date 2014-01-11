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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_HH
# define HPP_CORE_CONFIGURATION_SHOOTER_HH

namespace hpp {
  namespace core {
    /// Abstraction of configuration shooter
    ///
    /// Configuration shooters are used by random sampling algorithms to
    /// generate new configurations
    class HPP_CORE_DLLAPI ConfigurationShooter
    {
    public:
      ConfigurationShooter ()
	{
	}
      /// Shoot a random configuration
      virtual ConfigurationPtr_t shoot () const = 0;
    }; // class
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_CONFIGURATION_SHOOTER_HH
