// Copyright (c) 2019 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_OBSTACLE_USER_HH
# define HPP_CORE_OBSTACLE_USER_HH

# include <hpp/fcl/collision_data.h>

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/relative-motion.hh>
# include <hpp/core/collision-pair.hh>

namespace hpp {
  namespace core {
    /// Abstract class for handling obstacles
    ///
    /// Several classes perform collision detection between the bodies
    /// of a robot and a set of rigid-body obstacles of the environment.
    ///
    /// This class defines a common abstract interface for those classes.
    class HPP_CORE_DLLAPI ObstacleUserInterface
    {
      public:
        virtual ~ObstacleUserInterface () = default;

        /// Add an obstacle
        /// \param object obstacle added
        virtual void addObstacle (const CollisionObjectConstPtr_t& object) = 0;

        /// Remove a collision pair between a joint and an obstacle
        /// \param joint that holds the inner objects,
        /// \param obstacle to remove.
        virtual void removeObstacleFromJoint(const JointPtr_t& joint,
            const CollisionObjectConstPtr_t& object) = 0;

        /// Filter collision pairs.
        ///
        /// Remove pairs of object that cannot be in collision.
        /// This effectively disables collision detection between objects that
        /// have no possible relative motion due to the constraints.
        ///
        /// \param relMotion square symmetric matrix of RelativeMotionType of size numberDof x numberDof
        virtual void filterCollisionPairs (const RelativeMotion::matrix_type& relMotion) = 0;

        /// Set different security margins for collision pairs
        ///
        /// This function works joint-wise. If you need a finer control, use
        /// \ref setSecurityMarginBetweenBodies
        ///
        /// This method enables users to choose different security margins
        /// for each pair of robot joint or each pair robot joint - obstacle.
        /// \sa hpp::fcl::CollisionRequest::security_margin.
        virtual void setSecurityMargins(const matrix_t& securityMatrix) = 0;


        /// Set security margin for collision pair between the two bodies.
        ///
        /// \sa hpp::fcl::CollisionRequest::security_margin.
        virtual void setSecurityMarginBetweenBodies(const std::string& body_a,
                                                    const std::string& body_b,
                                                    const value_type& margin) = 0;
    }; // class ObstacleUserInterface

    /// Vector of validation class instances.
    ///
    /// \tparam Derived must be a shared point to some validation class. For
    ///         instance \link ConfigValidation ConfigValidationPtr_t \endlink
    ///         or \link PathValidation PathValidationPtr_t \endlink.
    ///
    /// This class implements the abstract interface defined by
    /// ObstacleUserInterface and stores a vector of path or config validation
    /// class instances. Methods
    /// \link ObstacleUserVector::addObstacle addObstacle \endlink,
    /// \link ObstacleUserVector::removeObstacleFromJoint
    ///       removeObstacleFromJoint\endlink, and
    /// \link ObstacleUserVector::filterCollisionPairs filterCollisionPairs
    /// \endlink iteratively dynamic cast each instance into
    /// Obstacle user interface and calls the derived implementation in case of
    /// success.
    template<typename Derived>
    class HPP_CORE_DLLAPI ObstacleUserVector : public ObstacleUserInterface
    {
      public:
        virtual ~ObstacleUserVector () = default;

        /// Add obstacle to each element
        ///
        /// Dynamic cast into ObstacleUserInterface each element
        /// of \link ObstacleUserVector::validations_ validations_
        /// \endlink and call method addObstacle of element.
        void addObstacle (const CollisionObjectConstPtr_t& object)
        {
          for (std::size_t i = 0; i < validations_.size(); ++i) {
            shared_ptr<ObstacleUserInterface> oui =
              HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, validations_[i]);
            if (oui) oui->addObstacle (object);
          }
        }

        /// Remove obstacle from joint to each element
        ///
        /// Dynamic cast into ObstacleUserInterface each element
        /// of \link ObstacleUserVector::validations_ validations_
        /// \endlink and call method removeObstacleFromJoint of element.
        void removeObstacleFromJoint(const JointPtr_t& joint,
            const CollisionObjectConstPtr_t& object)
        {
          for (std::size_t i = 0; i < validations_.size(); ++i) {
            shared_ptr<ObstacleUserInterface> oui =
              HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, validations_[i]);
            if (oui) oui->removeObstacleFromJoint (joint, object);
          }
        }

        /// Filter collision pairs to each element
        ///
        /// Dynamic cast into ObstacleUserInterface each element
        /// of \link ObstacleUserVector::validations_ validations_
        /// \endlink and call method filterCollisionPairs of element.
        void filterCollisionPairs (const RelativeMotion::matrix_type& relMotion)
        {
          for (std::size_t i = 0; i < validations_.size(); ++i) {
            shared_ptr<ObstacleUserInterface> oui =
              HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, validations_[i]);
            if (oui) oui->filterCollisionPairs (relMotion);
          }
        }

        /// \copydoc ObstacleUserInterface::setSecurityMargins
        void setSecurityMargins(const matrix_t& securityMatrix)
        {
          for (std::size_t i = 0; i < validations_.size(); ++i) {
            shared_ptr<ObstacleUserInterface> oui =
              HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, validations_[i]);
            if (oui) oui->setSecurityMargins (securityMatrix);
          }
        }

        /// \copydoc ObstacleUserInterface::setSecurityMarginBetweenBodies
        void setSecurityMarginBetweenBodies(const std::string& body_a,
                                            const std::string& body_b,
                                            const value_type& margin)
        {
          for (std::size_t i = 0; i < validations_.size(); ++i) {
            shared_ptr<ObstacleUserInterface> oui =
              HPP_DYNAMIC_PTR_CAST(ObstacleUserInterface, validations_[i]);
            if (oui)
              oui->setSecurityMarginBetweenBodies (body_a, body_b, margin);
          }
        }

        // Clear the vector of config validations
        void clear ()
        {
          validations_.clear ();
        }

      protected:
        typedef Derived value_t;
        typedef std::vector<value_t> values_t;

        ObstacleUserVector() = default;
        ObstacleUserVector(std::initializer_list<value_t> validations) :
          validations_ (validations)
        {};

        values_t validations_;
    }; // class ObstacleUserVector

    /// Stores a set of obstacles (movable or static).
    class HPP_CORE_DLLAPI ObstacleUser : public ObstacleUserInterface
    {
      public:
        virtual ~ObstacleUser () = default;

        static bool collide (const CollisionPairs_t& pairs,
            CollisionRequests_t& reqs,
            fcl::CollisionResult& res,
            std::size_t& i,
            pinocchio::DeviceData& data);

        // Get pairs checked for collision
        const CollisionPairs_t& pairs () const
        {
          return cPairs_;
        }

        // Get pairs checked for collision
        CollisionPairs_t& pairs ()
        {
          return cPairs_;
        }

        // Get requests of collision pairs
        const CollisionRequests_t& requests () const
        {
          return cRequests_;
        }

        // Get requests of collision pairs
        CollisionRequests_t& requests ()
        {
          return cRequests_;
        }

        fcl::CollisionRequest& defaultRequest()
        {
          return defaultRequest_;
        }

        void setRequests (const fcl::CollisionRequest& r);

        /// Add an obstacle
        /// \param object obstacle added
        virtual void addObstacle (const CollisionObjectConstPtr_t& object);

        /// Add an obstacle to a specific joint
        /// \param object obstacle added
        /// \param joint concerned with obstacle addition
        /// \param includeChildren whether to add obstacle to joint children
        /// Store obstacle and build a collision pair with each body of the robot.
        virtual void addObstacleToJoint (const CollisionObjectConstPtr_t& object,
                                         const JointPtr_t& joint, const bool includeChildren);

        /// Remove a collision pair between a joint and an obstacle
        /// \param joint that holds the inner objects,
        /// \param obstacle to remove.
        virtual void removeObstacleFromJoint(const JointPtr_t& joint,
            const CollisionObjectConstPtr_t& object);

        /// Filter collision pairs.
        ///
        /// Remove pairs of object that cannot be in collision.
        /// This effectively disables collision detection between objects that
        /// have no possible relative motion due to the constraints.
        /// \todo Before disabling collision pair, check if there is a collision.
        ///
        /// \param relMotion square symmetric matrix of RelativeMotionType of size numberDof x numberDof
        virtual void filterCollisionPairs (const RelativeMotion::matrix_type& relMotion);

        /// \copydoc ObstacleUserInterface::setSecurityMargins
        virtual void setSecurityMargins(const matrix_t& securityMatrix);

        /// \copydoc ObstacleUserInterface::setSecurityMarginBetweenBodies
        virtual void setSecurityMarginBetweenBodies(const std::string& body_a,
                                                    const std::string& body_b,
                                                    const value_type& margin);

      protected:
        /// Constructor of body pair collision
        ObstacleUser (DevicePtr_t robot)
          : robot_ (robot), defaultRequest_ (fcl::NO_REQUEST,1)
        {
          defaultRequest_.enable_cached_gjk_guess = true;
        }

        /// Copy constructor
        ObstacleUser (const ObstacleUser& other)
          : robot_ (other.robot_),
            defaultRequest_ (other.defaultRequest_),
            cPairs_ (other.cPairs_),
            pPairs_ (other.pPairs_),
            dPairs_ (other.dPairs_),
            cRequests_ (other.cRequests_),
            pRequests_ (other.pRequests_),
            dRequests_ (other.dRequests_)
        {}

        void addRobotCollisionPairs ();

        DevicePtr_t robot_;
        fcl::CollisionRequest defaultRequest_;

        CollisionPairs_t cPairs_, /// Active collision pairs
                         pPairs_, /// Parameterized collision pairs
                         dPairs_; /// Disabled collision pairs
        CollisionRequests_t cRequests_, /// Active collision requests
                            pRequests_, /// Parameterized collision requests
                            dRequests_; /// Disabled collision requests
    }; // class ObstacleUser
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_OBSTACLE_USER_HH
