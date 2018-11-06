//
// Copyright (c) 2018 CNRS
// Authors: Diane Bury
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_INITIALIZER_HH
# define HPP_CORE_CONTINUOUS_VALIDATION_INITIALIZER_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/continuous-validation.hh>
# include <pinocchio/multibody/geometry.hpp>

namespace hpp {
  namespace core {
    namespace
    {
      using continuousValidation::BodyPairCollisionPtr_t;
      using continuousValidation::SolidSolidCollision;
      using continuousValidation::BodyPairCollisions_t;

      typedef std::pair<se3::JointIndex, se3::JointIndex> JointIndexPair_t;

      struct JointIndexPairCompare_t
      {
        bool operator()(const JointIndexPair_t &p0, const JointIndexPair_t &p1) const
        {
          if (p0.first < p1.first)
            return true;
          if (p0.first > p1.first)
            return false;
          return (p0.second < p1.second);
        }
      };

      typedef std::map<JointIndexPair_t, BodyPairCollisionPtr_t, JointIndexPairCompare_t> BodyPairCollisionMap_t;
    } // namespace


    namespace continuousValidation {
      /// \addtogroup validation
      /// \{

      /// Initializer for the Continuous Validation
      ///
      /// This class initializes the continuous validation
      /// By default, it creates all the inner body pair collision of the robot
      ///
      ///
      class Initializer
      {
      public:
        static InitializerPtr_t create ()
        {
          Initializer* ptr = new Initializer ();
          InitializerPtr_t shPtr (ptr);
          return shPtr;
        }

        /// Set the pointer to the Continuous Validation
        ///
        /// \param continuousVal weak pointer to the continuous validation object
        void initContinuousValidation (ContinuousValidationWkPtr_t continuousVal)
        {
          continuousVal_ = continuousVal;
        }

        /// Initialize the Continuous Validation
        ///
        /// Call the default initialization that create all the
        /// inner body pair for collision checking.
        /// Override this method to change the initialization, for example
        /// if a new IntervalValidation is used.
        virtual void initialize()
        {
          defaultInitialize();
        }

        /// Reset the continuous validation to the "before initialized" state
        virtual void reset()
        {
          defaultReset();
        }

      protected:
        // Weak pointer to the continuous validation to initialize
        ContinuousValidationWkPtr_t continuousVal_;

        /// Create and store all inner body pair collisions for the robot
        ///
        void generateAutoCollisions()
        {
          ContinuousValidationPtr_t continuousVal = continuousVal_.lock ();
          DevicePtr_t robot = continuousVal->robot_;
          const se3::GeometryModel &gmodel = robot->geomModel();
          JointPtr_t joint1, joint2;
          BodyPairCollisionMap_t bodyPairMap;
          for (std::size_t i = 0; i < gmodel.collisionPairs.size(); ++i)
          {
            const se3::CollisionPair &cp = gmodel.collisionPairs[i];
            JointIndexPair_t jp(gmodel.geometryObjects[cp.first].parentJoint,
                                gmodel.geometryObjects[cp.second].parentJoint);

            // Ignore pairs of bodies that are in the same joint.
            if (jp.first == jp.second)
              continue;

            BodyPairCollisionMap_t::iterator _bp = bodyPairMap.find(jp);

            if (_bp == bodyPairMap.end())
            {
              joint1 = JointPtr_t(new Joint(robot, jp.first));
              joint2 = JointPtr_t(new Joint(robot, jp.second));
              continuousVal->bodyPairCollisions_.push_back(
                  SolidSolidCollision::create(joint2, joint1, continuousVal->tolerance_));
              bodyPairMap[jp] = continuousVal->bodyPairCollisions_.back();
            }
            CollisionObjectConstPtr_t co1 (new pinocchio::CollisionObject(robot, cp.first));
            CollisionObjectConstPtr_t co2 (new pinocchio::CollisionObject(robot, cp.second));
            bodyPairMap[jp]->addCollisionPair( co1,co2 );
          }
        }

        /// Constructor of continuous validation initializer
        ///
        Initializer ()
        {
        }

        /// Default initialization
        ///
        /// Generate inner body pair for collision checking
        void defaultInitialize()
        {
          generateAutoCollisions();
        }

        /// Default reset
        ///
        /// Reset the BodyPairCollision vector of the Continuous Validation
        void defaultReset()
        {
          ContinuousValidationPtr_t continuousVal = continuousVal_.lock ();
          continuousVal->bodyPairCollisions_.clear();
        }

      }; // class Initializer
      /// \}
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_INITIALIZER_HH
