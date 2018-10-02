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

# include <limits>
# include <iterator>

# include <boost/icl/continuous_interval.hpp>
# include <boost/icl/interval_set.hpp>

# include <hpp/fcl/collision_data.h>
# include <hpp/fcl/collision.h>
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/collision-object.hh>
# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/interpolated-path.hh>
# include <hpp/core/deprecated.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>
# include "continuous-validation/intervals.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {
      /// Computation of collision-free sub-intervals of a path
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
        /// \return true if the body pair is collision free for this parameter
        ///         value, false if the body pair is in collision.
        /// \note object should be in the positions defined by the configuration
        ///       of parameter t on the path.
        bool validateConfiguration (const value_type& t, interval_t& interval,
                  CollisionValidationReportPtr_t& report)
        {
          namespace icl = boost::icl;
          using std::numeric_limits;

          if (valid_) {
            interval = path_->timeRange ();
            return true;
          }
          continuous_interval iclInterval (interval.first, interval.second,
            icl::interval_bounds::closed());
          if (icl::contains (validInterval_, iclInterval))
          {
            // TODO interval could probably be enlarge using validInterval_
            // interval = validInterval_;
            return true;
          }

          value_type distanceLowerBound;
          if (!computeDistanceLowerBound(distanceLowerBound, report))
          {
            return false;
          }

          value_type halfLengthDist, halfLengthTol;
          /// \todo A finer bound could be computed when path is an
          ///       InterpolatedPath using the maximal velocity on each
          ///       subinterval
          if (distanceLowerBound == numeric_limits <value_type>::infinity ()) {
            halfLengthDist = numeric_limits <value_type>::infinity ();
            halfLengthTol = 0;
          } else {
            value_type Vm;
            halfLengthDist = collisionFreeInterval(t, distanceLowerBound, Vm);
            halfLengthTol = 2*tolerance_/Vm;
          }
          assert (!isnan (halfLengthDist));
          assert (!isnan (halfLengthTol));
          interval.first = t - (halfLengthDist + halfLengthTol);
          interval.second = t + (halfLengthDist + halfLengthTol);
          validInterval_.insert (continuous_interval(interval.first,
            interval.second, icl::interval_bounds::closed()));
          // Check if the whole path is valid.
          iclInterval = continuous_interval (path_->timeRange ().first,
            path_->timeRange ().second, icl::interval_bounds::closed());
          if (icl::contains (validInterval_, iclInterval))
            valid_ = true;
          return true;
        }

        // Get pairs checked for collision
        const CollisionPairs_t& pairs () const
        {
          return pairs_;
        }

        // Get maximal velocity
        value_type maximalVelocity () const
        {
          return maximalVelocity_;
        }

        virtual const JointPtr_t& joint_a () const { return JointPtr_t(); }
        virtual const JointPtr_t& joint_b () const { return JointPtr_t(); }
        virtual bool removeObjectTo_b (const CollisionObjectConstPtr_t& object) { return false;}
        virtual void addCollisionPair (const CollisionObjectConstPtr_t& left,
            const CollisionObjectConstPtr_t right) {}

      protected:
        mutable vector_t Vb_;
        CollisionPairs_t pairs_;
        /// Constructor of inter-body collision checking
        ///
        /// \param body_a, body_b bodies to test for collision
        /// \param tolerance allowed penetration should be positive
        /// \pre body_a and body_b should not be nul pointers.
        BodyPairCollision (value_type tolerance):
          IntervalValidation(tolerance), pairs_ (), maximalVelocity_(0)
        {
        }

      private:
        value_type maximalVelocity_;
        virtual value_type computeMaximalVelocity(vector_t& Vb) const = 0;

        virtual void setupPath()
        {
          if (HPP_DYNAMIC_PTR_CAST(StraightPath, path_)) refine_ = false;
          Vb_ = vector_t (path_->outputDerivativeSize());
          value_type t0 = path_->timeRange ().first;
          value_type t1 = path_->timeRange ().second;
          assert (t1 >= t0);
          if (t1 - t0 == 0) {
            maximalVelocity_ = std::numeric_limits<value_type>::infinity();
            refine_ = false;
          } else {
            path_->velocityBound (Vb_, t0, t1);
            maximalVelocity_ = computeMaximalVelocity (Vb_);
          }
        }

        /// Compute a collision free interval around t given a lower bound of
        /// the distance to obstacle.
        /// \retval maxVelocity
        /// \return the interval half length
        value_type collisionFreeInterval(const value_type &t,
                                        const value_type &distanceLowerBound,
                                        value_type &maxVelocity) const
        {
          value_type T[3], Vm[2];
          value_type tm, tM;
          maxVelocity = maximalVelocity_;
          T[0] = distanceLowerBound / maxVelocity;
          if (!refine_)
            return T[0];
          else
          {
            tm = t - T[0];
            tM = t + T[0];
            bool  leftIsValid = (tm < path_->timeRange().first );
            bool rightIsValid = (tM > path_->timeRange().second);
            if (leftIsValid && rightIsValid)
              return T[0];

            for (int i = 0; i < 2; ++i)
            {
              tm = t - (leftIsValid ? 0 : T[i]);
              tM = t + (rightIsValid ? 0 : T[i]);
              path_->velocityBound(Vb_, tm, tM);
              Vm[i] = computeMaximalVelocity(Vb_);
              T[i + 1] = distanceLowerBound / Vm[i];
            }
            assert(maximalVelocity_ >= Vm[1] && Vm[1] >= Vm[0] && T[1] >= T[2] && T[2] >= T[0]);
            // hppDout (info, "Refine changed the interval length of " << T[0] / T[2] << ", from " << T[0] << " to " << T[2]);
            maxVelocity = Vm[1];
            return T[2];
          }
        }

        /// Compute a lower bound of the distance between the bodies
        /// \retval distanceLowerBound
        /// \retval report the collision validation report
        /// \return true if the bodies are not in collision, else false
        bool computeDistanceLowerBound(value_type &distanceLowerBound,
          CollisionValidationReportPtr_t& report)
        {
          using std::numeric_limits;
          distanceLowerBound = numeric_limits <value_type>::infinity ();
          static const fcl::CollisionRequest request (1, false, true, 1, false,
            true, fcl::GST_INDEP);
          for (CollisionPairs_t::const_iterator _pair = pairs_.begin();
              _pair != pairs_.end(); ++_pair) {
            pinocchio::FclConstCollisionObjectPtr_t object_a = _pair->first ->fcl ();
            pinocchio::FclConstCollisionObjectPtr_t object_b = _pair->second->fcl ();
            fcl::CollisionResult result;
            fcl::collide (object_a, object_b, request, result);
            // Get result
            if (result.isCollision ()) {
              report = CollisionValidationReportPtr_t
                (new CollisionValidationReport);
              report->object1 = _pair->first ;
              report->object2 = _pair->second;
              report->result = result;
              return false;
            }
            if (result.distance_lower_bound < distanceLowerBound) {
              distanceLowerBound = result.distance_lower_bound;
            }
          }
          return true;
        }

      }; // class BodyPairCollision
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_BODY_PAIR_COLLISION_HH
