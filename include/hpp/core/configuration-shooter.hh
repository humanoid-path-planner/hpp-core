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

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup configuration_sampling
    /// \{

    /// Abstraction of configuration shooter
    ///
    /// Configuration shooters are used by random sampling algorithms to
    /// generate new configurations
    class HPP_CORE_DLLAPI ConfigurationShooter
    {
    public:
      /// Shoot a random configuration
      virtual ConfigurationPtr_t shoot () const
      {
        ConfigurationPtr_t q (new Configuration_t);
        shoot (*q);
        return q;
      }

      /// Shoot a random configuration
      /// \param q the configuration (resized if necessary).
      ///
      /// \deprecated This method is virtual for backward compatibility. It will
      /// become non-virtual in the future. Child classes should rather implement
      /// \ref impl_shoot so that both prototype of method shoot remain available.
      virtual void shoot (Configuration_t& q) const { impl_shoot(q); }

      virtual ~ConfigurationShooter () {};
    protected:
      ConfigurationShooter ()
    {
    }
      /// Store weak pointer to itself
      void init (const ConfigurationShooterWkPtr_t& weak)
    {
  weakPtr_ = weak;
    }

      virtual void impl_shoot (Configuration_t& q) const = 0;
    private:
      ConfigurationShooterWkPtr_t weakPtr_;
    }; // class
  } //   namespace core
  /// \}
} // namespace hpp
#endif // HPP_CORE_CONFIGURATION_SHOOTER_HH
