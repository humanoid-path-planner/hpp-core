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

#ifndef HPP_CORE_DISTANCE_HH
# define HPP_CORE_DISTANCE_HH

# include <hpp/model/fwd.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Abstract class for distance between configurations
    class HPP_CORE_DLLAPI Distance {
    public:
      virtual value_type operator () (ConfigurationIn_t q1,
				  ConfigurationIn_t q2) const
      {
	return impl_distance (q1, q2);
      }

      virtual DistancePtr_t clone () const = 0;
      
    protected:

      Distance ()
	{
	}
      /// Derived class should implement this function
      virtual value_type impl_distance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const = 0;
    }; // class Distance
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_DISTANCE_HH
