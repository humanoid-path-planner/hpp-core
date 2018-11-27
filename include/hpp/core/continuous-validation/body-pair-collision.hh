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

# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      /// Computation of collision-free sub-intervals of a path. 
      ///
      /// This class aims at validating a path for the absence of collision
      /// between two bodies of a robot, which are in most common cases
      /// two solid bodies (see SolidSolidCollision). With this abstraction,
      /// other bodies (like cables) can also be checked for collision.
      ///
      /// The interval of definition of the path is successively covered
      /// by intervals where boths bodies are proved to be collision-free.
      /// Each interval is computed by bounding from above the velocity of
      /// all points of body 1 in the reference frame of body 2.
      ///
      /// See <a href="continuous-collision-checking.pdf"> this document </a>
      /// for details.
      class BodyPairCollision : public IntervalValidation<CollisionValidationReportPtr_t>
      {
      public:
        typedef std::pair<CollisionObjectConstPtr_t, CollisionObjectConstPtr_t> CollisionPair_t;
        typedef std::vector<CollisionPair_t> CollisionPairs_t;

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
                  CollisionValidationReportPtr_t& report,
                  pinocchio::DeviceData& data);

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

        virtual std::ostream& print (std::ostream& os) const = 0;

      protected:
        /// Constructor of body pair collision
        ///
        /// \param tolerance allowed penetration should be positive
        BodyPairCollision (value_type tolerance):
          IntervalValidation(tolerance), m_ (new Model), maximalVelocity_(0)
        {
        }

        virtual void setReport (CollisionValidationReportPtr_t& report,
                            fcl::CollisionResult result,
                            CollisionPair_t _pair) const
        {
          report = CollisionValidationReportPtr_t
            (new CollisionValidationReport);
          report->object1 = _pair.first ;
          report->object2 = _pair.second;
          report->result = result;
        }

      private:
        struct Model {
          CollisionPairs_t pairs;
        };
        boost::shared_ptr<Model> m_;

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
        bool computeDistanceLowerBound(value_type &distanceLowerBound,
          CollisionValidationReportPtr_t& report, pinocchio::DeviceData& data) const;

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
