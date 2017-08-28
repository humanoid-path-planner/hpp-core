//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_DISTANCE_REEDS_SHEPP_HH
# define HPP_CORE_DISTANCE_REEDS_SHEPP_HH

# include <hpp/core/distance.hh>
# include <hpp/core/steering-method/fwd.hh>

namespace hpp {
  namespace core {
    namespace distance {
      /// \addtogroup steering_method
      /// \{

      /// Reeds and Shepp distance
      ///
      /// Compute the distance between two configurations of a nonholonomic
      /// cart-like mobile robot with bounded curvature.
      class HPP_CORE_DLLAPI ReedsShepp : public Distance
      {
      public:
	virtual DistancePtr_t clone () const;
	static ReedsSheppPtr_t create (const Problem& problem);
	static ReedsSheppPtr_t create (const Problem& problem,
				     const value_type& turningRadius,
				     JointPtr_t xyJoint, JointPtr_t rzJoint);

	static ReedsSheppPtr_t createCopy (const ReedsSheppPtr_t& distance);
      protected:
	ReedsShepp (const Problem& problem);
	ReedsShepp (const Problem& problem,
		    const value_type& turningRadius,
		    JointPtr_t xyJoint, JointPtr_t rzJoint);
	ReedsShepp (const ReedsShepp& distance);
	
	/// Derived class should implement this function
	virtual value_type impl_distance (ConfigurationIn_t q1,
					  ConfigurationIn_t q2) const;
	void init (const ReedsSheppWkPtr_t& weak);
      private:
	steeringMethod::ReedsSheppPtr_t sm_;
	ReedsSheppWkPtr_t weak_;
      }; // class ReedsShepp
    /// \}
    } // namespace distance
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_DISTANCE_REEDS_SHEPP_HH
