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
# include <hpp/core/obstacle-user.hh>
# include <hpp/core/path.hh>
# include <hpp/core/path-validation.hh>
# include <hpp/core/path-validation-report.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>

namespace hpp {
  namespace core {
    using continuousValidation::IntervalValidation;
    using continuousValidation::IntervalValidationPtr_t;
      /// \addtogroup validation
      /// \{

      /// Continuous validation of a path
      ///
      /// This class valides a path for various criteria.
      /// The default validation criterion is the absence of collisions
      /// between bodies of a robot and the
      /// environment.
      ///
      /// Validation of PathVector instances is performed path by path.
      ///
      /// A path is valid if and only if each interval validation element
      /// is valid along the whole interval of definition (class
      /// continuousValidation::IntervalValidation).
      ///
      /// In order to validate other criteria, users can add their own
      /// derivation of class continuousValidation::IntervalValidation using
      /// method \link ContinuousValidation::addIntervalValidation
      /// addIntervalValidation\endlink. They can also make use of two types
      /// of delegates:
      /// \li \link ContinuousValidation::Initialize Initialize\endlink
      ///     and user defined derived classes. Instances of these classes
      ///     may be added to the list of initializers by calling method
      ///     \link ContinuousValidation::add <ContinuousValidation::Initialize>
      ///     add <Initialize> \endlink. Upon calling method
      /// \link ContinuousValidation::initialize initialize\endlink, methods
      /// \c doExecute of these instances are called sucessively.
      /// \li \link ContinuousValidation::AddObstacle AddObstacle\endlink
      ///     and user defined derived classes. Instances of these classes
      ///     may be added to the list of obstacle adders by calling method
      ///     \link ContinuousValidation::add <ContinuousValidation::AddObstacle>
      ///     add <AddObstacle> \endlink. Upon calling method
      /// \link ContinuousValidation::addObstacle addObstacle\endlink, method
      /// \c doExecute of these instances are called sucessively.
      ///
      /// Base class ContinuousValidation::Initialize initializes
      /// collision pairs between bodies of the robot.
      ///
      /// Validation of a collision pair is based
      /// on the computation of an upper-bound of the relative velocity of
      /// objects of one joint (or of the environment) in the reference frame
      /// of the other joint. This is implemented in
      /// continuousValidation::BodyPairCollision and
      /// continuousValidation::SolidSolidCollision.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details on the continuous collision checking.
      ///
      /// See <a href="continuous-validation.pdf"> this document </a>
      /// for details on the architecture of the code.
    class HPP_CORE_DLLAPI ContinuousValidation :
      public PathValidation,
      public ObstacleUserInterface
    {
    public:
      /// Delegate class to initialize ContinuousValidation instance
      ///
      /// See method ContinuousValidation::initialize for details
      class Initialize
      {
      public:
        Initialize(ContinuousValidation& owner);
        /// Initialize collision pairs between bodies of the robot
        /// Iterate over all collision pairs of the robot, and for each one,
        /// \li create an instance of continuousValidation::SolidSolidCollision,
        /// \li add it to class ContinuousValidation using method
        ///     ContinuousValidation::addIntervalValidation.
        virtual void doExecute() const;
        virtual ~Initialize () {}
      protected:
        ContinuousValidation& owner_;
      }; // class Initialize
      /// Delegate class to add obstacle to ContinuousValidation instance
      ///
      /// See method ContinuousValidation::addObstacle for details
      class AddObstacle
      {
      public:
        AddObstacle(ContinuousValidation& owner);
        /// Add an obstacle
        /// \param object obstacle added
        /// Add the object to each collision pair a body of which is the
        /// environment.
        virtual void doExecute(const CollisionObjectConstPtr_t &object) const;
        virtual ~AddObstacle () {}
      protected:
        ContinuousValidation& owner_;
        DeviceWkPtr_t robot_;
      }; // class AddObstacle

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
      /// Iteratively call method doExecute of delegate classes AddObstacle
      /// \param object new obstacle.
      /// \sa ContinuousValidation::add, ContinuousValidation::AddObstacle.
      virtual void addObstacle (const CollisionObjectConstPtr_t& object);

      /// Remove a collision pair between a joint and an obstacle
      /// \param joint the joint that holds the inner objects,
      /// \param obstacle the obstacle to remove.
      /// \note collision configuration validation needs to know about
      /// obstacles. This virtual method does nothing for configuration
      /// validation methods that do not care about obstacles.
      virtual void removeObstacleFromJoint
	(const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle);

      /// Filter collision pairs.
      ///
      /// Remove pairs of object that cannot be in collision.
      /// This effectively disables collision detection between objects that
      /// have no possible relative motion due to the constraints.
      ///
      /// \param relMotion square symmetric matrix of RelativeMotionType of size numberDof x numberDof
      void filterCollisionPairs (const RelativeMotion::matrix_type& relMotion);

      /// Set different security margins for collision pairs
      ///
      /// This method enables users to choose different security margins
      /// for each pair of robot body or each pair robot body - obstacle.
      /// \sa hpp::fcl::CollisionRequest::security_margin.
      virtual void setSecurityMargins(const matrix_t& securityMatrix);

      /// \name Delegate
      /// \{

      /// Add a delegate
      /// \tparam Delegate type of delegate.
      /// \param instance of delegate class.
      ///
      /// Delegates are used to enable users to specialize some actions of the
      /// class without deriving the class. This class supports two types of
      /// delegates:
      /// \li Initialize: method
      ///     \link ContinuousValidation::Initialize::doExecute doExecute
      ///     \endlink of instances of this class and of user defined derived
      ///     classes are called successively upon call of method
      ///     \link ContinuousValidation::initialize\endlink,
      /// \li AddObstacle: method
      ///     \link ContinuousValidation::AddObstacle::doExecute doExecute
      ///     \endlink of instances of this class and of user defined derived
      ///     classes are called successively upon call of method
      ///     \link ContinuousValidation::addObstacle\endlink,
      template <class Delegate> void add(const Delegate& delegate);

      /// Reset delegates of a type
      /// \tparam delegate type of delegate
      template <class Delegate> void reset();

      /// \}
      /// Add interval validation instance
      void addIntervalValidation
        (const IntervalValidationPtr_t& intervalValidation);
      /// Get tolerance value
      value_type tolerance () const
      {
        return tolerance_;
      }

      DevicePtr_t robot() const
      {
        return robot_;
      }
      /// Iteratively call method doExecute of delegate classes Initialize
      /// \sa ContinuousValidation::add, ContinuousValidation::Initialize.
      void initialize();

      virtual ~ContinuousValidation ();
    protected:
      typedef continuousValidation::IntervalValidations_t IntervalValidations_t;

      static void setPath(IntervalValidations_t& intervalValidations,
          const PathPtr_t &path, bool reverse);

      /// Constructor
      /// \param robot the robot for which validation is performed,
      /// \param tolerance maximal penetration allowed.
      ContinuousValidation (const DevicePtr_t& robot,
				   const value_type& tolerance);

      /// Validate interval centered on a path parameter
      /// \param intervalValidations collision to consider
      /// \param config Configuration at abscissa t on the path.
      /// \param t parameter value in the path interval of definition
      /// \retval interval interval over which the path is collision-free,
      ///                  not necessarily included in definition interval
      /// \return true if the body pair is collision free for this parameter
      ///         value, false if the body pair is in collision.
      /// \note object should be in the positions defined by the configuration
      ///       of parameter t on the path.
      virtual bool validateConfiguration
        (IntervalValidations_t& intervalValidations,
          const Configuration_t& config,
          const value_type& t,
          interval_t& interval,
          PathValidationReportPtr_t& report);

      /// Validate a set of intervals for a given parameter along a path
      ///
      /// \tparam IntervalValidation type of container of validation elements
      ///         (for instance validation for collision between a pair of
      ///         bodies),
      /// \tparam ValidationReportTypePtr_t type of validation report produced
      ///         in case non validation. Should derive from ValidationReport.
      /// \param objects able to validate an interval for collision,
      /// \param t center of the interval to be validated,
      /// \retval interval interval validated for all objects,
      /// \retval smallestInterval iterator to the validation element that
      ///         returned the smallest interval.
      bool validateIntervals
        (IntervalValidations_t& validations, const value_type &t,
         interval_t &interval, PathValidationReportPtr_t &pathReport,
         typename IntervalValidations_t::iterator& smallestInterval,
         pinocchio::DeviceData& data)
      {
        typename IntervalValidations_t::iterator itMin = validations.begin();
        for (IntervalValidations_t::iterator itVal (validations.begin());
             itVal != validations.end(); ++itVal)
        {
          ValidationReportPtr_t report;
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

      DevicePtr_t robot_;
      value_type tolerance_;

      /// Store weak pointer to itself.
      void init (ContinuousValidationWkPtr_t weak);

      /// All BodyPairValidation to validate
      IntervalValidations_t intervalValidations_;
      /// BodyPairCollision for which collision is disabled
      IntervalValidations_t disabledBodyPairCollisions_;

      pinocchio::Pool<IntervalValidations_t> bodyPairCollisionPool_;

      value_type stepSize_;
    private:
      // Weak pointer to itself
      ContinuousValidationWkPtr_t weak_;

      virtual bool validateStraightPath
        (IntervalValidations_t& intervalValidations,
          const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
          PathValidationReportPtr_t& report) = 0;

      std::vector<Initialize> initialize_;
      std::vector<AddObstacle> addObstacle_;
    }; // class ContinuousValidation
    /// \}
    template <>
    void ContinuousValidation::add<ContinuousValidation::AddObstacle>
    (const ContinuousValidation::AddObstacle& delegate);

    template <>
    void ContinuousValidation::reset<ContinuousValidation::AddObstacle>();

    template <>
    void ContinuousValidation::add<ContinuousValidation::Initialize>
    (const ContinuousValidation::Initialize& delegate);

    template <>
    void ContinuousValidation::reset<ContinuousValidation::Initialize>();

    template <class Delegate> void ContinuousValidation::add
    (const Delegate& delegate)
    {
      assert (false &&
              "No delegate of this type in class ContinuousValidation.");
    }
    template <class Delegate> void ContinuousValidation::reset()
    {
      assert (false &&
              "No delegate of this type in class ContinuousValidation.");
    }

  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_HH
