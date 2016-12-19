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

#ifndef HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH
# define HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH

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
	JointConstPtr_t joint_;
	value_type value_;
      }; // struct CoefficientVelocity
      typedef std::vector <CoefficientVelocity> CoefficientVelocities_t;

      struct PathVelocity
      {
	typedef std::map <value_type, value_type> Velocities_t;

	PathVelocity (CoefficientVelocities_t const* coefs, PathPtr_t path) :
	  coefs_ (coefs)
	{
	  StraightPathPtr_t sp = HPP_DYNAMIC_PTR_CAST (StraightPath, path);
	  if (sp) { init (sp); return; }
	  InterpolatedPathPtr_t ip = HPP_DYNAMIC_PTR_CAST (InterpolatedPath,
							   path);
	  if (ip) { init (ip); return; }
	  throw std::logic_error ("Continuous collision checking: unknown type"
				  " of paths");
	}

	PathVelocity (CoefficientVelocities_t const* coefs) :
	  maximalVelocity_ (0), coefs_ (coefs) {}

	void init (StraightPathPtr_t path)
	{
	  value_type t0 = path->timeRange ().first;
	  value_type t1 = path->timeRange ().second;
	  Configuration_t q0 = path->initial();
	  Configuration_t q1 = path->end();
	  maximalVelocity_ = computeMaximalVelocity (t0, q0, t1, q1);
	  maximalVelocities_.insert(std::make_pair (t1, maximalVelocity_));
	}

	void init (InterpolatedPathPtr_t path)
	{
	  typedef InterpolatedPath::InterpolationPoints_t IPs_t;
	  const IPs_t& ips = path->interpolationPoints();
	  value_type tprev = path->timeRange ().first;
	  Configuration_t qprev = path->initial();
	  maximalVelocity_ = 0;
	  for (IPs_t::const_iterator it = (++ips.begin ());
	       it != ips.end(); ++it) {
	    const value_type& t = it->first;
	    const Configuration_t& q = it->second;
	    value_type mv = computeMaximalVelocity (tprev, qprev, t, q);
	    maximalVelocities_.insert(std::make_pair (t, mv));
	    if (mv > maximalVelocity_) maximalVelocity_ = mv;
	    tprev = t;
	    qprev = q;
	  }
	}

	value_type maximalVelocity (const value_type& t) const
	{
	  Velocities_t::const_iterator itAfter =
	    maximalVelocities_.lower_bound(t);
	  if (itAfter != maximalVelocities_.begin ()) itAfter--;
	  return itAfter->second;
	}

	/// Compute maximal velocity of points of body1 in the frame of body 2
	/// \param path input path
	value_type computeMaximalVelocity (const value_type& t0,
					   ConfigurationIn_t q0,
					   const value_type& t1,
					   ConfigurationIn_t q1)
	{
	  const value_type T = t1 - t0;
	  if (T == 0) return std::numeric_limits<value_type>::infinity();

	  value_type maximalVelocity = 0;
	  for (CoefficientVelocities_t::const_iterator itCoef =
		 coefs_->begin (); itCoef != coefs_->end (); ++itCoef) {
	    const JointConstPtr_t& joint = itCoef->joint_;
	    const value_type& value = itCoef->value_;
	    maximalVelocity += value * joint->configuration ()->distance
	      (q0, q1, joint->rankInConfiguration ()) / T;
	  }
	  return maximalVelocity;
	}

	Velocities_t maximalVelocities_;
	value_type maximalVelocity_;
	CoefficientVelocities_t const* coefs_;
      }; // struct PathVelocity
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp


#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_HH
