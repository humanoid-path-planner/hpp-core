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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH
# define HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH

# include <hpp/core/continuous-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      /// \addtogroup validation
      /// \{

      /// Continuous validation of a path
      ///
      /// This class tests for collision
      /// \li straight paths, or
      /// \li concatenation of straight paths (PathVector).
      ///
      /// A path is valid if and only if each interval validation element
      /// is valid along the whole interval of definition.
      ///
      /// For each interval validation element, a union of sub-intervals
      /// where the element is valid is computed.
      ///
      /// First, each validation element is tested at the beginning of the interval
      /// (at the end if reverse is set to true). Then the element that
      /// has the smaller upper bound of the first valid sub-interval is
      /// tested at the middle of the segment delimited by the upper bound
      /// of the first valid sub-interval and by the lower bound of the second
      /// valid sub-interval (or the end of the interval of definition if the
      /// union of sub-intervals contains only one sub-interval).
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
      class HPP_CORE_DLLAPI Dichotomy : public ContinuousValidation
      {
      public:
        /// Create instance and return shared pointer
        /// \param robot the robot for which continuous validation is performed,
        /// \param tolerance maximal penetration allowed.
        static DichotomyPtr_t
          create (const DevicePtr_t& robot, const value_type& tolerance);

        virtual ~Dichotomy ();
      protected:
        /// Constructor
        /// \param robot the robot for which continuous validation is performed,
        /// \param tolerance maximal penetration allowed.
        Dichotomy (const DevicePtr_t& robot, const value_type& tolerance);
        /// Store weak pointer to itself
        void init(const DichotomyWkPtr_t weak);
      private:
        // Weak pointer to itself
        DichotomyWkPtr_t weak_;
        bool validateStraightPath (BodyPairCollisions_t& bodyPairCollisions,
            const PathPtr_t& path, bool reverse,
            PathPtr_t& validPart,
            PathValidationReportPtr_t& report);
        template <bool reverse>
        bool validateStraightPath (BodyPairCollisions_t& bodyPairCollisions,
            const PathPtr_t& path,
            PathPtr_t& validPart,
            PathValidationReportPtr_t& report);
      }; // class Dichotomy
    } // namespace continuousValidation
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_DICHOTOMY_HH
