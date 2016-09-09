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

#ifndef HPP_CORE_DISTANCE_BETWEEN_OBJECTS_HH
# define HPP_CORE_DISTANCE_BETWEEN_OBJECTS_HH

# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// Computation of distances between pairs of objects
    class DistanceBetweenObjects
    {
    public:
      DistanceBetweenObjects (const DevicePtr_t& robot);
      /// Add an obstacle
      /// \param object obstacle to add
      /// Create distance computation pairs for each body of the robot
      void addObstacle (const CollisionObjectConstPtr_t &object);
      /// Add a list of obstacles
      void obstacles (const ObjectStdVector_t &obstacles);
      /// Compute distances between pairs of objects stored in bodies
      void computeDistances ();
      /// Get collision pairs
      const CollisionPairs_t&
	collisionPairs () const {return collisionPairs_;};
      /// Get result of distance computations
      const DistanceResults_t&
	distanceResults () const {return distanceResults_;};
      /// \}


    private:
      DevicePtr_t robot_;
      CollisionPairs_t collisionPairs_;
      DistanceResults_t distanceResults_;
    };
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_DISTANCE_BETWEEN_OBJECTS_HH
