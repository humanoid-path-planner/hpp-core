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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH

# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      HPP_PREDEF_CLASS (BodyPairCollision);
      typedef boost::shared_ptr <BodyPairCollision> BodyPairCollisionPtr_t;
      typedef std::vector <BodyPairCollisionPtr_t> BodyPairCollisions_t;
    } // namespace continuousCollisionChecking
      /// \addtogroup validation
      /// \{

      /// Continuous validation of a path for collision
      ///
      /// This class is derived into two sub-classes that test for collision
      /// \li a StraightPath, or
      /// \li a PathVector containing StraightPath or PathVector instances
      ///
      /// A path is valid if and only if each pair of objects to test is
      /// collision-free along the whole interval of definition. 
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
    class HPP_CORE_DLLAPI ContinuousCollisionChecking : public PathValidation
    {
    public:
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
      virtual void addObstacle (const CollisionObjectConstPtr_t& object);

      /// Remove a collision pair between a joint and an obstacle
      /// \param the joint that holds the inner objects,
      /// \param the obstacle to remove.
      /// \notice collision configuration validation needs to know about
      /// obstacles. This virtual method does nothing for configuration
      /// validation methods that do not care about obstacles.
      virtual void removeObstacleFromJoint
	(const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle);

      virtual ~ContinuousCollisionChecking ();
    protected:
      /// Constructor
      /// \param robot the robot for which collision checking is performed,
      /// \param tolerance maximal penetration allowed.
      ContinuousCollisionChecking (const DevicePtr_t& robot,
				   const value_type& tolerance);
      bool validateConfiguration (const Configuration_t& config,
				  bool reverse, value_type& tmin,
				  PathValidationReportPtr_t& report);
      DevicePtr_t robot_;
      value_type tolerance_;
      continuousCollisionChecking::BodyPairCollisions_t bodyPairCollisions_;
      value_type stepSize_;
    private:
      virtual bool validateStraightPath
	(const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
	 PathValidationReportPtr_t& report) = 0;
    }; // class ContinuousCollisionChecking
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH
