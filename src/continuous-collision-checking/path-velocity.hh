//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PATH_VELOCITY_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PATH_VELOCITY_HH

# include <boost/mpl/vector.hpp>
# include <boost/mpl/for_each.hpp>

# include <pinocchio/multibody/geometry.hpp>

# include <hpp/core/path/spline.hh>

namespace hpp {
  namespace core {
    namespace continuousCollisionChecking {
      /// Multiplicative coefficients of linear and angular velocities
      struct CoefficientVelocity
      {
	CoefficientVelocity () : value_ (0)
	{
	}
	/// Joint the degrees of freedom of which the bounds correspond to.
	JointPtr_t joint_;
	value_type value_;
      }; // struct CoefficientVelocity
      typedef std::vector <CoefficientVelocity> CoefficientVelocities_t;

      struct PathVelocity
      {
	PathVelocity (CoefficientVelocities_t const* coefs, PathPtr_t path) :
	  coefs_ (coefs), refine_ (true),
          path_ (path), Vb_ (path->outputDerivativeSize())
	{
          if (HPP_DYNAMIC_PTR_CAST(StraightPath, path_)) refine_ = false;

          value_type t0 = path->timeRange ().first;
          value_type t1 = path->timeRange ().second;
          assert (t1 >= t0);
          if (t1 - t0 == 0) maximalVelocity_ = std::numeric_limits<value_type>::infinity();
          else {
            path_->velocityBound (Vb_, t0, t1);
            maximalVelocity_ = computeMaximalVelocity (Vb_);
          }
	}

	PathVelocity (CoefficientVelocities_t const* coefs) :
	  maximalVelocity_ (0), coefs_ (coefs) {}

	/// Compute maximal velocity of points of body1 in the frame of body 2
	/// \param path input path
	value_type computeMaximalVelocity (vectorIn_t v) const
	{
	  value_type maximalVelocity = 0;
	  for (CoefficientVelocities_t::const_iterator itCoef =
		 coefs_->begin (); itCoef != coefs_->end (); ++itCoef) {
	    const JointPtr_t& joint = itCoef->joint_;
	    const value_type& value = itCoef->value_;
	    maximalVelocity += value * v.segment(joint->rankInVelocity(), joint->numberDof()).norm();
	  }
	  return maximalVelocity;
	}

        /// Compute a collision free interval around t given a lower bound of
        /// the distance to obstacle.
        /// \retval maxVelocity
        /// \return the interval half length
        value_type collisionFreeInterval(const value_type& t,
            const value_type& distanceLowerBound,
            value_type& maxVelocity) const
        {
          value_type T[3];
          value_type tm, tM;
          maxVelocity = maximalVelocity_;
          T[0] = distanceLowerBound / maxVelocity;
          if (!refine_
              || ( t - T[0] < path_->timeRange().first
                && t + T[0] < path_->timeRange().second))
            return T[0];
          else {
            for (int i = 0; i < 2; ++i) {
              tm = t - T[i]; if (tm < path_->timeRange().first ) tm = t;
              tM = t + T[i]; if (tM > path_->timeRange().second) tM = t;
              path_->velocityBound (Vb_, tm, tM);
              maxVelocity = computeMaximalVelocity(Vb_);
              T[i+1] = distanceLowerBound / maxVelocity;
            }
            assert(T[2] <= T[1]);
            return T[2];
          }
        }

	value_type maximalVelocity_;
	CoefficientVelocities_t const* coefs_;
        bool refine_;
        PathPtr_t path_;
        mutable vector_t Vb_;
      }; // struct PathVelocity
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp


#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PATH_VELOCITY_HH
