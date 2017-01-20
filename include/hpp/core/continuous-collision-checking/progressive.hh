//
// Copyright (c) 2014, 2015, 2016, 2017 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH

# include <hpp/core/continuous-collision-checking.hh>

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      /// \addtogroup validation
      /// \{

      /// Continuous validation of a path for collision
      ///
      /// This class tests for collision
      /// \li straight paths, or
      /// \li concatenation of straight paths.
      ///
      /// A path is valid if and only if each pair of objects to test is
      /// collision-free along the whole interval of definition. 
      ///
      /// For each pair, a union of sub-intervals where the pair is
      /// collision-free is computed.
      ///
      /// The validation of a path is progressive, starting at the beginning
      /// of the interval (or at the end if reverse is set to true).
      /// The smallest valid sub-interval for all pairs centered at the current
      /// parameter is computed. The current parameter is thus set to the upper
      /// bound of this sub-interval and the validation process goes on until
      /// a collision is detected or the current parameter reaches the end of
      /// the interval of definition.
      ///
      /// Collision pairs between bodies of the robot are initialized at
      /// construction of the instance.
      ///
      /// Method addObstacle adds an obstacle in the environment.
      /// For each joint, a new pair is created with the new obstacle.
      ///
      /// Validation of pairs along straight interpolations is based on the
      /// computation of an upper-bound of the relative velocity of objects
      /// of one joint (or of the environment) in the reference frame of the
      /// other joint.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details.
      class HPP_CORE_DLLAPI Progressive : public ContinuousCollisionChecking
      {
      public:
	/// Create instance and return shared pointer
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	static ProgressivePtr_t
	  create (const DevicePtr_t& robot, const value_type& tolerance);

	virtual ~Progressive ();
      protected:
	/// Constructor
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	Progressive (const DevicePtr_t& robot, const value_type& tolerance);
      private:
	virtual bool validateStraightPath (const PathPtr_t& path, bool reverse,
					   PathPtr_t& validPart,
					   PathValidationReportPtr_t& report);
      }; // class Progressive
    } // namespace continuousCollisionChecking
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH
