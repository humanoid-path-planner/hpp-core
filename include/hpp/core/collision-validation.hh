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

#ifndef HPP_CORE_COLLISION_VALIDATION_HH
# define HPP_CORE_COLLISION_VALIDATION_HH

# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/config-validation.hh>
# include <hpp/fcl/collision_data.h>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// Validate a configuration with respect to collision
    ///
    class HPP_CORE_DLLAPI CollisionValidation : public ConfigValidation
    {
    public:
      static CollisionValidationPtr_t create (const DevicePtr_t& robot);

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation. If non valid,
      ///         a validation report will be allocated and returned via this
      ///         shared pointer.
      /// \return whether the whole config is valid.
      virtual bool validate (const Configuration_t& config,
			     ValidationReportPtr_t& validationReport);

      /// Add an obstacle
      /// \param object obstacle added
      /// Store obstacle and build a collision pair with each body of the robot.
      virtual void addObstacle (const CollisionObjectPtr_t& object);

      /// Remove a collision pair between a joint and an obstacle
      /// \param the joint that holds the inner objects,
      /// \param the obstacle to remove.
      /// \notice collision configuration validation needs to know about
      /// obstacles. This virtual method does nothing for configuration
      /// validation methods that do not care about obstacles.
      virtual void removeObstacleFromJoint
	(const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle);

      void filterCollisionPairs (const RelativeMotion::matrix_type& matrix);

      void checkParameterized (bool active)
      {
        checkParameterized_ = active;
      }

      bool checkParameterized () const
      {
        return checkParameterized_;
      }

      /// Rearrange the collisions pairs of all configValidation in a random manner
      /// \brief randomnizeCollisionPairs
      ///
      virtual void randomnizeCollisionPairs();
    public:
      /// fcl low level request object used for collision checking.
      /// modify this attribute to obtain more detailed validation
      /// reports in a call to validate.
      fcl::CollisionRequest collisionRequest_;
    protected:
      CollisionValidation (const DevicePtr_t& robot);
      DevicePtr_t robot_;
    private:
      CollisionPairs_t collisionPairs_,
                       parameterizedPairs_,
                       disabledPairs_;

      bool checkParameterized_;
    }; // class ConfigValidation
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COLLISION_VALIDATION_HH
