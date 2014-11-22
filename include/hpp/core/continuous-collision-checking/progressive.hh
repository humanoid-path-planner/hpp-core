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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH

# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      namespace progressive {
	HPP_PREDEF_CLASS (BodyPairCollision);
	typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;
	typedef std::list <BodyPairCollisionPtr_t> BodyPairCollisions_t;
      }

      /// Continuous validation of a path for collision
      class HPP_CORE_DLLAPI Progressive : public PathValidation
      {
      public:
	/// Create instance and return shared pointer
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	static ProgressivePtr_t
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
			       PathPtr_t& validPart);
 	/// Compute a valid interval starting from the path beginning
	///
	/// \param path the path to check for validity,
	/// \param reverse if true check from the end,
	/// \retval the extracted valid part of the path, pointer to path if
	///         path is valid.
	/// \return whether the whole path is valid.
	/// \retval validationReport information about the validation process:
	///         which objects have been detected in collision and at which
	///         parameter along the path.
	/// \precond validationReport should be a of type
	///          CollisionPathValidationReport.
	virtual bool validate (const PathPtr_t& path, bool reverse,
			       PathPtr_t& validPart,
			       ValidationReport& validationReport);

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

	virtual ~Progressive ();
      protected:
	/// Constructor
	/// \param robot the robot for which collision checking is performed,
	/// \param tolerance maximal penetration allowed.
	Progressive (const DevicePtr_t& robot,
		   const value_type& tolerance);
      private:
	bool validateConfiguration (const Configuration_t& config,
				    bool reverse, value_type& tmin,
				    CollisionPathValidationReport& report);
	DevicePtr_t robot_;
	value_type tolerance_;
	progressive::BodyPairCollisions_t bodyPairCollisions_;
      value_type stepSize_;
      /// This member is used by the validate method that does not take a
      /// validation report as input to call the validate method that expects
      /// a validation report as input. This is not fully satisfactory, but
      /// I did not find a better solution.
      CollisionPathValidationReport unusedReport_;
      }; // class Progressive
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PROGRESSIVE_HH
