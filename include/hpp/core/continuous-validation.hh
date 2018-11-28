//
// Copyright (c) 2014, 2015, 2016, 2017, 2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_HH
# define HPP_CORE_CONTINUOUS_VALIDATION_HH

# include <hpp/pinocchio/pool.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/path-validation.hh>
# include <hpp/core/path-validation-report.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>

namespace hpp {
  namespace core {
      /// \addtogroup validation
      /// \{

      /// Continuous validation of a path
      ///
      /// This class tests for collision
      /// \li straight paths, or
      /// \li concatenation of straight paths (PathVector).
      ///
      /// A path is valid if and only if each interval validation element
      /// is valid along the whole interval of definition
      ///
      /// Interval validation elements can be pairs of objects to test for
      /// collision (BodyPairCollision) or an other type of validation.
      ///
      /// Collision pairs between bodies of the robot are initialized at
      /// construction of the instance through the Initializer.
      ///
      /// Method addObstacle adds an obstacle in the environment.
      /// For each joint, a new pair is created with the new obstacle.
      ///
      /// Validation of a collision pair along straight interpolations is based
      /// on the computation of an upper-bound of the relative velocity of
      /// objects of one joint (or of the environment) in the reference frame
      /// of the other joint. This is implemented in BodyPairCollision and
      /// SolidSolidCollision.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details on the continuous collision checking.
      ///
      /// See <a href="continuous-validation.pdf"> this document </a>
      /// for details on the architecture of the code.
    class HPP_CORE_DLLAPI ContinuousValidation : public PathValidation
    {
    public:
      /// Compute the largest valid interval starting from the path beginning
      ///
      /// \param path the path to check for validity,
      /// \param reverse if true check from the end,
      /// \retval validPart the extracted valid part of the path, pointer to path if
      ///         path is valid.
      /// \retval report information about the validation process. A report
      ///         is allocated if the path is not valid.
      /// \return true if the whole path is valid.
      virtual bool validate (const PathPtr_t& path, bool reverse,
			     PathPtr_t& validPart,
			     PathValidationReportPtr_t& report);
      /// Add an obstacle
      /// \param object obstacle added
      /// Add the object to each collision pair a body of which is the
      /// environment.
      /// care about obstacles.
      virtual void addObstacle (const CollisionObjectConstPtr_t& object);

      /// Remove a collision pair between a joint and an obstacle
      /// \param joint the joint that holds the inner objects,
      /// \param obstacle the obstacle to remove.
      /// \note collision configuration validation needs to know about
      /// obstacles. This virtual method does nothing for configuration
      /// validation methods that do not care about obstacles.
      virtual void removeObstacleFromJoint
	(const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle);

      void filterCollisionPairs (const RelativeMotion::matrix_type& relMotion);

      /// Change the initializer
      /// The continuous validation is then reset and the new initializer is
      /// called to do the new initialization
      void changeInitializer (continuousValidation::InitializerPtr_t initializer);

      virtual ~ContinuousValidation ();
    protected:
      typedef continuousValidation::BodyPairCollisions_t BodyPairCollisions_t;

      static void setPath(BodyPairCollisions_t& bodyPairCollisions,
          const PathPtr_t &path, bool reverse);

      /// Constructor
      /// \param robot the robot for which validation is performed,
      /// \param tolerance maximal penetration allowed.
      ContinuousValidation (const DevicePtr_t& robot,
				   const value_type& tolerance);

      /// Validate interval centered on a path parameter
      /// \param bodyPairCollisions collision to consider
      /// \param config Configuration at abscissa t on the path.
      /// \param t parameter value in the path interval of definition
      /// \retval interval interval over which the path is collision-free,
      ///                  not necessarily included in definition interval
      /// \return true if the body pair is collision free for this parameter
      ///         value, false if the body pair is in collision.
      /// \note object should be in the positions defined by the configuration
      ///       of parameter t on the path.
      virtual bool validateConfiguration (BodyPairCollisions_t& bodyPairCollisions,
          const Configuration_t& config,
          const value_type& t,
          interval_t& interval,
          PathValidationReportPtr_t& report);

      DevicePtr_t robot_;
      value_type tolerance_;

      /// Store weak pointer to itself.
      void init (ContinuousValidationWkPtr_t weak);

      // all BodyPairValidation to validate
      BodyPairCollisions_t bodyPairCollisions_;
      // BodyPairCollision for which collision is disabled
      BodyPairCollisions_t disabledBodyPairCollisions_;

      pinocchio::Pool<BodyPairCollisions_t> bodyPairCollisionPool_;

      value_type stepSize_;
      // Initializer as a delegate
      continuousValidation::InitializerPtr_t initializer_;
    private:
      // Weak pointer to itself
      ContinuousValidationWkPtr_t weak_;

      virtual bool validateStraightPath (BodyPairCollisions_t& bodyPairCollisions,
          const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
          PathValidationReportPtr_t& report) = 0;

      /// Validate a set of intervals for a given parameter along a path
      ///
      /// \tparam IntervalValidations type of container of validation elements
      ///         (for instance validation for collision between a pair of
      ///         bodies),
      /// \tparam ValidationReportTypePtr_t type of validation report produced
      ///         in case non validation. Should derive from ValidationReport.
      /// \param objects able to validate an interval for collision,
      /// \param t center of the interval to be validated,
      /// \retval interval interval validated for all objects,
      /// \retval smallestInterval iterator to the validation element that
      ///         returned the smallest interval.
      template<typename IntervalValidations, typename ValidationReportTypePtr_t>
      bool validateIntervals
        (IntervalValidations& validations, const value_type &t,
         interval_t &interval, PathValidationReportPtr_t &pathReport,
         typename IntervalValidations::iterator& smallestInterval,
         pinocchio::DeviceData& data)
      {
        typename IntervalValidations::iterator itMin = validations.begin();
        for (typename IntervalValidations::iterator itVal = validations.begin();
            itVal != validations.end(); ++itVal)
        {
          ValidationReportTypePtr_t report;
          // the valid interval will not be greater than "interval" so we do not
          // need to perform validation on a greater interval.
          interval_t tmpInt = interval;
          if (!(*itVal)->validateConfiguration(t, tmpInt, report, data))
          {
            pathReport = PathValidationReportPtr_t(new PathValidationReport);
            pathReport->configurationReport = report;
            pathReport->parameter = t;
            return false;
          }
          else
          {
            if (interval.second > tmpInt.second)
            {
              itMin = itVal;
              smallestInterval = itVal;
            }
            interval.first = std::max(interval.first, tmpInt.first);
            interval.second = std::min(interval.second, tmpInt.second);
            assert((*itVal)->path()->length() == 0 || interval.second > interval.first);
            assert(interval.first <= t);
            assert(t <= interval.second);
          }
        }
        return true;
      }
      friend class continuousValidation::Initializer;

    }; // class ContinuousValidation
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_HH
