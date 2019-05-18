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

# include <hpp/pinocchio/fwd.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/node.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Abstract class for distance between configurations
    class HPP_CORE_DLLAPI Distance {
    public:
      value_type operator () (ConfigurationIn_t q1,
				  ConfigurationIn_t q2) const
      {
	return impl_distance (q1, q2);
      }

      value_type operator () (NodePtr_t n1, NodePtr_t n2) const
      {
	return impl_distance (n1, n2);
      }

      value_type compute (ConfigurationIn_t q1,
				  ConfigurationIn_t q2) const
      {
	return impl_distance (q1, q2);
      }

      value_type compute (NodePtr_t n1, NodePtr_t n2) const
      {
	return impl_distance (n1, n2);
      }

      virtual DistancePtr_t clone () const = 0;

      virtual ~Distance () {};
    protected:

      Distance ()
	{
	}
      /// Derived class should implement this function
      virtual value_type impl_distance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const = 0;
      virtual value_type impl_distance (NodePtr_t n1, NodePtr_t n2) const
      {
        return impl_distance (*n1->configuration(), *n2->configuration());
      }
    }; // class Distance
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_DISTANCE_HH
