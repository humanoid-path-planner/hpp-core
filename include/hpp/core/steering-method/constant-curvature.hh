//
// Copyright (c) 2017 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_CONSTANT_CURVATURE_HH
# define HPP_CORE_STEERING_METHOD_CONSTANT_CURVATURE_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>
# include <hpp/core/steering-method/fwd.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// Path of constant curvature for a carlike robot
      class ConstantCurvature : public Path {
      public:
        typedef Path parent_t;
        virtual ~ConstantCurvature () throw () {}
         /// Create instance and return shared pointer.
        /// \param robot the carlike robot,
        /// \param init, end Initial and final configurations of the path,
        /// \param curveLength distance traveled by the middle of the rear
        ///        wheel axis, negative values correspond to backward motions
        /// \param pathLength length of the interval of definition. Should be
        ///        positive,
        /// \param curvature curvature of the path,
        /// \param xyId id of degrees of freedom corresponding to (x,y)
        /// coordinates of robot,
        /// \param rzId id of degrees of freedom corresponding to orientation
        /// of robot.
        /// \param rz joint corresponding to orientation of robot,
        /// \param wheels vector of joints corresponding to wheels,
        /// \param constraints set of contraints the path is suject to.
        static ConstantCurvaturePtr_t create
        (const DevicePtr_t& robot, ConfigurationIn_t init,
         ConfigurationIn_t end,
         value_type curveLength, value_type pathLength,
         value_type curvature, size_type xyId, size_type rzId,
         const JointPtr_t rz, const std::vector<JointPtr_t> wheels,
         const ConstraintSetPtr_t& constraints);

         /// Create instance and return shared pointer.
        static ConstantCurvaturePtr_t createCopy
        (const ConstantCurvaturePtr_t& other);
 
        /// Create instance and return shared pointer.
        static ConstantCurvaturePtr_t createCopy
        (const ConstantCurvaturePtr_t& other,
         const ConstraintSetPtr_t& constraints);
 
        /// Return a shared pointer to a copy of this
        virtual PathPtr_t copy () const;

        /// Return a shared pointer to a copy of this with constraints
        /// \param constraints constraints to apply to the copy
        /// \pre *this should not have constraints.
        virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
        {
	  return createCopy (weak_.lock (), constraints);
        }
      
       /// Get the initial configuration
        inline Configuration_t initial () const
        {
          return initial_;
        }

        /// Get the final configuration
        inline Configuration_t end () const
        {
          return end_;
        }

      protected:
        /// Print path in a stream
        virtual std::ostream& print (std::ostream &os) const;

        /// Constructor
        /// \param robot the carlike robot,
        /// \param init, end Initial and final configurations of the path,
        /// \param curveLength distance traveled by the middle of the rear
        ///        wheel axis, negative values correspond to backward motions
        /// \param pathLength length of the interval of definition. Should be
        ///        positive,
        /// \param curvature curvature of the path,
        /// \param xyId id of degrees of freedom corresponding to (x,y)
        /// coordinates of robot,
        /// \param rzId id of degrees of freedom corresponding to orientation
        /// of robot.
        /// \param rz joint corresponding to orientation of robot,
        /// \param wheels vector of joints corresponding to wheels.
        ConstantCurvature (const DevicePtr_t& robot, ConfigurationIn_t init,
                           ConfigurationIn_t end,
                           value_type curveLength, value_type pathLength,
                           value_type curvature, size_type xyId,
                           size_type rzId, const JointPtr_t rz,
                           const std::vector<JointPtr_t> wheels);

        /// Constructor
        /// \param robot the carlike robot,
        /// \param init, end Initial and final configurations of the path,
        /// \param curveLength distance traveled by the middle of the rear
        ///        wheel axis, negative values correspond to backward motions
        /// \param pathLength length of the interval of definition. Should be
        ///        positive,
        /// \param curvature curvature of the path,
        /// \param xyId id of degrees of freedom corresponding to (x,y)
        /// coordinates of robot,
        /// \param rzId id of degrees of freedom corresponding to orientation
        /// of robot.
        /// \param rz joint corresponding to orientation of robot,
        /// \param wheels vector of joints corresponding to wheels,
        /// \param constraints set of contraints the path is suject to.
        ConstantCurvature (const DevicePtr_t& robot, ConfigurationIn_t init,
                           ConfigurationIn_t end,
                           value_type curveLength, value_type pathLength,
                           value_type curvature, size_type xyId, size_type rzId,
                           const JointPtr_t rz,
                           const std::vector<JointPtr_t> wheels,
                           ConstraintSetPtr_t constraints);

        /// Copy constructor
        ConstantCurvature (const ConstantCurvature& other);

        /// Copy constructor with constraints
        ConstantCurvature (const ConstantCurvature& other,
                           const ConstraintSetPtr_t& constraints);

        virtual bool impl_compute (ConfigurationOut_t result,
                                   value_type param) const;
        /// Virtual implementation of derivative
        virtual void impl_derivative
        (vectorOut_t result, const value_type& param, size_type order) const;
        /// Virtual implementation of velocity bound
        virtual void impl_velocityBound (vectorOut_t bound,
                                         const value_type& param0,
                                         const value_type& param1) const;

        /// Virtual implementation of path extraction
        virtual PathPtr_t impl_extract (const interval_t& paramInterval) const
          throw (projection_error);

        /// store weak pointer to itself
        void init (const ConstantCurvatureWkPtr_t& weak)
        {
          parent_t::init (weak);
          weak_ = weak;
        }
    private:
        /// Set the wheel joints for a car-like vehicle.
        ///
        /// \param rz joint from which the turning radius was computed.
        /// \param wheels bounded rotation joints.
        void setWheelJoints (const JointPtr_t rz,
                             const std::vector<JointPtr_t> wheels);

        const DevicePtr_t robot_;
        Configuration_t initial_;
        Configuration_t end_;
        value_type curveLength_;
        const value_type curvature_;
        const size_type xyId_,rzId_;
        size_type dxyId_,drzId_;
        value_type forward_;
        struct Wheels_t {
          value_type value; // Constant value of the wheel angle
          JointPtr_t j;
          Wheels_t () : j () {}
        };
        std::vector<Wheels_t> wheels_;
        ConstantCurvatureWkPtr_t weak_;
      }; // class ConstantCurvature
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_STEERING_METHOD_CONSTANT_CURVATURE_HH
