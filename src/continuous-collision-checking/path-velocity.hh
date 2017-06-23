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
	typedef std::map <value_type, value_type> Velocities_t;
        template <typename PathPtr> struct init {};
        struct initializer {
          PathPtr_t p;
          PathVelocity& pv;
          bool& ok;
          initializer (PathPtr_t _p, PathVelocity& _pv, bool& _ok) : p(_p), pv(_pv), ok(_ok) {}
          template <typename T> void operator() (boost::shared_ptr<T>) {
            if (ok) return;
            boost::shared_ptr<T> tp = HPP_DYNAMIC_PTR_CAST (T, p);
            if (tp) { init<T>::run (pv, tp); ok = true; }
          }
        };

	PathVelocity (CoefficientVelocities_t const* coefs, PathPtr_t path) :
	  coefs_ (coefs)
	{
          bool ok = false;
          boost::mpl::for_each< boost::mpl::vector<
            StraightPathPtr_t,
            InterpolatedPathPtr_t,
            typename path::Spline<path::BernsteinBasis, 1>::Ptr_t,
            typename path::Spline<path::BernsteinBasis, 2>::Ptr_t,
            typename path::Spline<path::BernsteinBasis, 3>::Ptr_t,
            typename path::Spline<path::CanonicalPolynomeBasis, 1>::Ptr_t,
            typename path::Spline<path::CanonicalPolynomeBasis, 2>::Ptr_t,
            typename path::Spline<path::CanonicalPolynomeBasis, 3>::Ptr_t
            > > (initializer (path, *this, ok));
          if (!ok) throw std::logic_error ("Unknown type of paths");
	}

	PathVelocity (CoefficientVelocities_t const* coefs) :
	  maximalVelocity_ (0), coefs_ (coefs) {}

	value_type maximalVelocity (const value_type& t) const
	{
	  Velocities_t::const_iterator itAfter =
	    maximalVelocities_.lower_bound(t);
	  if (itAfter != maximalVelocities_.begin ()) itAfter--;
	  return itAfter->second;
	}

	/// Compute maximal velocity of points of body1 in the frame of body 2
	/// \param path input path
	value_type computeMaximalVelocity (
					   const value_type& t0, ConfigurationIn_t q0,
					   const value_type& t1, ConfigurationIn_t q1) const
	{
	  const value_type T = t1 - t0;
	  if (T == 0) return std::numeric_limits<value_type>::infinity();

	  value_type maximalVelocity = 0;
	  for (CoefficientVelocities_t::const_iterator itCoef =
		 coefs_->begin (); itCoef != coefs_->end (); ++itCoef) {
	    const JointPtr_t& joint = itCoef->joint_;
	    const value_type& value = itCoef->value_;
	    maximalVelocity += value * joint->jointModel().distance (q0, q1) / T;
	  }
	  return maximalVelocity;
	}

	Velocities_t maximalVelocities_;
	value_type maximalVelocity_;
	CoefficientVelocities_t const* coefs_;
      }; // struct PathVelocity

      template <> struct PathVelocity::init<StraightPath> {
        static void run (PathVelocity& pv, StraightPathPtr_t path)
        {
          value_type t0 = path->timeRange ().first;
          value_type t1 = path->timeRange ().second;
          Configuration_t q0 = path->initial();
          Configuration_t q1 = path->end();
          pv.maximalVelocity_ = pv.computeMaximalVelocity (t0, q0, t1, q1);
          pv.maximalVelocities_.insert(std::make_pair (t1, pv.maximalVelocity_));
        }
      };

      template <> struct PathVelocity::init<InterpolatedPath> {
        static void run (PathVelocity& pv, InterpolatedPathPtr_t path)
        {
          typedef InterpolatedPath::InterpolationPoints_t IPs_t;
          const IPs_t& ips = path->interpolationPoints();
          value_type tprev = path->timeRange ().first;
          Configuration_t qprev = path->initial();
          pv.maximalVelocity_ = 0;
          for (IPs_t::const_iterator it = (++ips.begin ());
              it != ips.end(); ++it) {
            const value_type& t = it->first;
            const Configuration_t& q = it->second;
            value_type mv = pv.computeMaximalVelocity (tprev, qprev, t, q);
            pv.maximalVelocities_.insert(std::make_pair (t, mv));
            if (mv > pv.maximalVelocity_) pv.maximalVelocity_ = mv;
            tprev = t;
            qprev = q;
          }
        }
      };

      template <int PB, int SP> struct PathVelocity::init<path::Spline<PB, SP> > {
        static void run (PathVelocity& pv, typename path::Spline<PB, SP>::Ptr_t path)
        {
          vector_t maxV (path->outputDerivativeSize());
          path->maxVelocity(maxV);

          pv.maximalVelocity_ = 0;
          if (path->length() == 0) {
            pv.maximalVelocity_ = std::numeric_limits<value_type>::infinity();
          } else {
            for (CoefficientVelocities_t::const_iterator itCoef =
                pv.coefs_->begin (); itCoef != pv.coefs_->end (); ++itCoef) {
              const JointPtr_t& joint = itCoef->joint_;
              const value_type& value = itCoef->value_;
              pv.maximalVelocity_ += value * maxV.segment(joint->rankInVelocity(), joint->numberDof()).norm();
            }
          }
          value_type t1 = path->timeRange ().second;
          pv.maximalVelocities_.insert(std::make_pair (t1, pv.maximalVelocity_));
        }
      };
    } // namespace continuousCollisionChecking
  } // namespace core
} // namespace hpp


#endif // HPP_CORE_CONTINUOUS_COLLISION_CHECKING_PATH_VELOCITY_HH
