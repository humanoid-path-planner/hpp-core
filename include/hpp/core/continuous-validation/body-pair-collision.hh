//
// Copyright (c) 2014,2015,2016,2018 CNRS
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_BODY_PAIR_COLLISION_HH
# define HPP_CORE_CONTINUOUS_VALIDATION_BODY_PAIR_COLLISION_HH

# include <boost/icl/continuous_interval.hpp>
# include <boost/icl/interval_set.hpp>

# include <hpp/fcl/collision_data.h>

# include <hpp/core/collision-pair.hh>
# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      /// Computation of collision-free sub-intervals of a path. 
      ///
      /// This class aims at validating a path for the absence of collision
      /// between two bodies of a robot, or between a robot body and the
      /// environment. Bodies are considered as rigid.
      ///
      /// If the bodies are part of an open kinematic chain, the
      /// computations are performed by class SolidSolidCollision.
      ///
      /// With this abstraction, other bodies (like legs of a parallel robot)
      /// can also be checked for collision. In this case, the specialized
      /// class needs to implement method \link
      /// BodyPairCollision::computeMaximalVelocity computeMaximalVelocity
      /// \endlink, taking into account the constrained motion of the legs
      /// implied by the closure of the kinematic chain.
      ///
      /// See <a href="continuous-validation.pdf"> this document </a>
      /// for details.
      class BodyPairCollision : public IntervalValidation
      {
      public:
        /// Validate interval centered on a path parameter
        /// \param t parameter value in the path interval of definition
        /// \param[in,out] interval as input, interval over which
              ///                collision checking must be performed.
              ///                As output, interval over which pair is collision-free,
              ///                not necessarily included in definition interval.
        /// \param report the collision validation report
        /// \return true if the body pair is collision free for this parameter
        ///         value, false if the body pair is in collision.
        /// \note object should be in the positions defined by the configuration
        ///       of parameter t on the path.
        bool validateConfiguration (const value_type& t, interval_t& interval,
                  ValidationReportPtr_t& report,
                  const pinocchio::DeviceData& data);

        // Get pairs checked for collision
        const CollisionPairs_t& pairs () const
        {
          return m_->pairs;
        }

        // Get pairs checked for collision
        CollisionPairs_t& pairs ()
        {
          return m_->pairs;
        }

        // Get maximal velocity
        value_type maximalVelocity () const
        {
          return maximalVelocity_;
        }

        /// Returns joint A index or -1 if no such joint exists.
        virtual size_type indexJointA () const { return -1; }
        /// Returns joint B index or -1 if no such joint exists.
        virtual size_type indexJointB () const { return -1; }

        virtual bool removeObjectTo_b (const CollisionObjectConstPtr_t& /*object*/) { return false;}
        virtual void addCollisionPair (const CollisionObjectConstPtr_t& /*left*/,
            const CollisionObjectConstPtr_t &/*right*/) {}

        virtual std::string name () const = 0;
        virtual std::ostream& print (std::ostream& os) const = 0;

        /// \name Security margin
        /// \{

        /// Get security margin
        value_type securityMargin() const
        {
          return collisionRequest_.security_margin;
        }
        /// Set security margin
        void securityMargin(const value_type& securityMargin)
        {
          collisionRequest_.security_margin = securityMargin;
        }
        /// \}
      protected:
        /// Constructor of body pair collision
        ///
        /// \param tolerance allowed penetration should be positive
        BodyPairCollision (value_type tolerance):
          IntervalValidation(tolerance), m_ (new Model),
          collisionRequest_(fcl::DISTANCE_LOWER_BOUND, 1), maximalVelocity_(0)
        {
          collisionRequest_.enable_cached_gjk_guess = true;
        }

        /// Copy constructor
        BodyPairCollision (const BodyPairCollision& other):
          IntervalValidation(other), m_(other.m_),
          collisionRequest_(other.collisionRequest_),
          maximalVelocity_(other.maximalVelocity_)
        {}

        virtual void setReport (ValidationReportPtr_t& report,
                            fcl::CollisionResult result,
                            CollisionPair_t _pair) const
        {
          report = CollisionValidationReportPtr_t
            (new CollisionValidationReport (_pair, result));
        }

      private:
        struct Model {
          CollisionPairs_t pairs;
        };
        shared_ptr<Model> m_;
        fcl::CollisionRequest collisionRequest_;

        mutable vector_t Vb_;
        value_type maximalVelocity_;

        /// Compute maximal velocity for a given velocity bound
        /// \param Vb velocity
        virtual value_type computeMaximalVelocity(vector_t& Vb) const = 0;

        /// Compute the maximal velocity along the path
        /// To be called after a new path has been set
        virtual void setupPath();

        /// Compute a collision free interval around t given a lower bound of
        /// the distance to obstacle.
        /// \param t the time in the path to test for a collision free interval
        /// \return distanceLowerBound the interval half length
        /// \retval maxVelocity the maximum velocity reached during this interval
        value_type collisionFreeInterval(const value_type &t,
                                        const value_type &distanceLowerBound,
                                        value_type &maxVelocity) const;

        /// Compute a lower bound of the distance between the bodies
        /// \retval distanceLowerBound
        /// \retval report the collision validation report
        /// \return true if the bodies are not in collision, else false
        virtual bool computeDistanceLowerBound(value_type &distanceLowerBound,
          ValidationReportPtr_t& report,
          const pinocchio::DeviceData& data);

      }; // class BodyPairCollision

      inline std::ostream& operator<< (std::ostream& os,
                                       const BodyPairCollision& pair)
      {
        return pair.print (os);
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_BODY_PAIR_COLLISION_HH
