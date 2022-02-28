//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
