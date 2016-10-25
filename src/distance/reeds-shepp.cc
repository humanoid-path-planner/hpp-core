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

#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>

namespace hpp {
  namespace core {
    namespace distance {

      DistancePtr_t ReedsShepp::clone () const
      {
	return createCopy (weak_.lock ());
      }

      ReedsSheppPtr_t ReedsShepp::create (const ProblemPtr_t& problem)
      {
	ReedsShepp* ptr (new ReedsShepp (problem));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      ReedsSheppPtr_t ReedsShepp::create (const ProblemPtr_t& problem,
					const value_type& turningRadius,
					size_type xyRank, size_type rzRank)
      {
	ReedsShepp* ptr (new ReedsShepp (problem, turningRadius,
					 xyRank, rzRank));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      ReedsSheppPtr_t ReedsShepp::createCopy (const ReedsSheppPtr_t& distance)
      {
	ReedsShepp* ptr (new ReedsShepp (*distance));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      ReedsShepp::ReedsShepp (const ProblemPtr_t& problem) :
	Distance (), sm_ (steeringMethod::ReedsShepp::createWithGuess (problem))
      {
      }

      ReedsShepp::ReedsShepp (const ProblemPtr_t& problem,
			      const value_type& turningRadius,
			      size_type xyRank, size_type rzRank) :
	Distance (), sm_ (steeringMethod::ReedsShepp::create
			  (problem, turningRadius, xyRank, rzRank))
      {
      }
      
      ReedsShepp::ReedsShepp (const ReedsShepp& distance) :
	Distance (), sm_ (steeringMethod::ReedsShepp::createCopy (distance.sm_))
      {
      }

      value_type ReedsShepp::impl_distance (ConfigurationIn_t q1,
					    ConfigurationIn_t q2) const
      {
	return (*sm_) (q1, q2)->length ();
      }

      void ReedsShepp::init (const ReedsSheppWkPtr_t& weak)
      {
	weak_ = weak;
      }
			  
    } // namespace distance
  } //   namespace core
} // namespace hpp
