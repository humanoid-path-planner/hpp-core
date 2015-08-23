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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_DICHOTOMY_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_DICHOTOMY_HH

# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      namespace dichotomy {
	HPP_PREDEF_CLASS (BodyPairCollision);
	typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;
	typedef std::list <BodyPairCollisionPtr_t> BodyPairCollisions_t;
      }
      using dichotomy::BodyPairCollisions_t;

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
      /// First, each pair is tested at the beginning of the interval
      /// (at the end if reverse is set to true). Then the pair that
      /// has the smaller upper bound of the first valid sub-interval is
      /// tested at the middle of the segment delimited by the upper bound
      /// of the first valid sub-interval and by the lower bound of the second
      /// valid sub-interval (or the end of the interval of definition if the
      /// union of sub-intervals contains only one sub-interval).
      ///
      /// Collision pairs between bodies of the robot are initialized at
      /// construction of the instance.
      ///
      /// Method Dichotomy::addObstacle adds an obstacle in the environment.
      /// This obstacle is added to the pair corresponding to each joint with
      /// the environment.
      ///
      /// Validation of pairs along straight interpolations is based on the
      /// computation of an upper-bound of the relative velocity of objects
      /// of one joint (or of the environment) in the reference frame of the
      /// other joint.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details.
      class HPP_CORE_DLLAPI Dichotomy : public PathValidation
      {
      public:
	/// Create instance and return shared pointer
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	static DichotomyPtr_t
	  create (const DevicePtr_t& robot, const value_type& tolerance);
	/// Compute a valid interval starting from the path beginning
	///
	/// \param path the path to check for validity,
	/// \param reverse if true check from the end,
	/// \retval the extracted valid part of the path, pointer to path if
	///         path is valid.
	/// \retval report information about the validation process. The type
	///         can be derived for specific implementation
	/// \return whether the whole path is valid.
	/// \precond validationReport should be a of type
	///          CollisionPathValidationReport.
	virtual bool validate (const PathPtr_t& path, bool reverse,
			       PathPtr_t& validPart) HPP_CORE_DEPRECATED;
	/// Compute a valid interval starting from the path beginning
	///
	/// \param path the path to check for validity,
	/// \param reverse if true check from the end,
	/// \retval the extracted valid part of the path, pointer to path if
	///         path is valid.
	/// \return whether the whole path is valid.
	/// \retval report information about the validation process.
	/// \precond validationReport should be a of type
	///          CollisionPathValidationReport.
	virtual bool validate (const PathPtr_t& path, bool reverse,
			       PathPtr_t& validPart,
			       ValidationReport& report) HPP_CORE_DEPRECATED;

	/// Compute the largest valid interval starting from the path beginning
	///
	/// \param path the path to check for validity,
	/// \param reverse if true check from the end,
	/// \retval the extracted valid part of the path, pointer to path if
	///         path is valid.
	/// \retval report information about the validation process. A report
	///         is allocated if the path is not valid.
	/// \return whether the whole path is valid.
	virtual bool validate (const PathPtr_t& path, bool reverse,
			       PathPtr_t& validPart,
			       PathValidationReportPtr_t& report);

	/// Add an obstacle
	/// \param object obstacle added
	/// Add the object to each collision pair a body of which is the
	/// environment.
	/// care about obstacles.
	virtual void addObstacle (const CollisionObjectPtr_t& object);

	/// Remove a collision pair between a joint and an obstacle
	/// \param the joint that holds the inner objects,
	/// \param the obstacle to remove.
	/// \notice collision configuration validation needs to know about
	/// obstacles. This virtual method does nothing for configuration
	/// validation methods that do not care about obstacles.
	virtual void removeObstacleFromJoint
	  (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle);

	virtual ~Dichotomy ();
      protected:
	/// Constructor
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	Dichotomy (const DevicePtr_t& robot,
		   const value_type& tolerance);
      private:
	DevicePtr_t robot_;
	value_type tolerance_;
	dichotomy::BodyPairCollisions_t bodyPairCollisions_;
      /// This member is used by the validate method that does not take a
      /// validation report as input to call the validate method that expects
      /// a validation report as input. This is not fully satisfactory, but
      /// I did not find a better solution.
      CollisionPathValidationReport unusedReport_;
      }; // class Dichotomy
    } // namespace continuousCollisionChecking
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_DICHOTOMY_HH
