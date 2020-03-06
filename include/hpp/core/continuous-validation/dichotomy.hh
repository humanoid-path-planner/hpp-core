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
      /// This class is a specialization of ContinuousValidation.
      ///
      /// The interval validation is performed by
      /// \li validating the beginning of the interval (or the end if paramater
      ///     reverse is set to true when calling
      ///     ContinuousValidation::validate),
      /// \li validate intervals centered at the middle of the biggest non
      ///     tested interval.
      /// See <a href="continuous-validation.pdf"> this document </a>
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
