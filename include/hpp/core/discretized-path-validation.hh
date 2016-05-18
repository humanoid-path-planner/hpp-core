//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_DISCRETIZED_PATH_VALIDATION_HH
# define HPP_CORE_DISCRETIZED_PATH_VALIDATION_HH

# include <hpp/core/path-validation.hh>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// Discretized validation of a path
    ///
    /// Apply some configuration validation algorithms at discretized values
    /// of the path parameter.
    class HPP_CORE_DLLAPI DiscretizedPathValidation : public PathValidation
    {
    public:
      static DiscretizedPathValidationPtr_t
	create (const DevicePtr_t& robot, const value_type& stepSize);

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

      /// Add a configuration validation object
      virtual void add (const ConfigValidationPtr_t& configValidation);

      /// Add an obstacle
      /// \param object obstacle added
      virtual void addObstacle (const CollisionObjectPtr_t&);

      /// Remove a collision pair between a joint and an obstacle
      /// \param joint the joint that holds the inner objects,
      /// \param obstacle the obstacle to remove.
      /// \notice collision configuration validation needs to know about
      /// obstacles. This virtual method does nothing for configuration
      /// validation methods that do not care about obstacles.
      virtual void removeObstacleFromJoint (const JointPtr_t& joint,
          const CollisionObjectPtr_t& obstacle);

      virtual void filterCollisionPairs (const RelativeMotion::matrix_type& matrix);

    protected:
      DiscretizedPathValidation
  (const DevicePtr_t& robot, const value_type& stepSize);

      value_type stepSize_;

    private:
      DevicePtr_t robot_;
      ConfigValidationsPtr_t configValidations_;
    }; // class DiscretizedPathValidation
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_DISCRETIZED_PATH_VALIDATION_HH
