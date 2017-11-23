//
// Copyright (c) 2014 CNRS
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

#include "extracted-path.hh"

#include <hpp/util/debug.hh>

#include <hpp/core/time-parameterization.hh>

namespace hpp {
  namespace core {
    namespace timeParameterization {
      class HPP_CORE_LOCAL Shift :
        public TimeParameterization
      {
        public:
          typedef boost::shared_ptr<Shift> Ptr_t;

          static TimeParameterizationPtr_t createWithCheck (TimeParameterizationPtr_t tp, value_type t, value_type s)
          {
            if (t == 0 && s == 0) return tp;
            else return create (tp, t, s);
          }

          static Ptr_t create (TimeParameterizationPtr_t tp, value_type t, value_type s)
          {
            Ptr_t shift = HPP_DYNAMIC_PTR_CAST(Shift, tp);
            if (shift)
              return Ptr_t(new Shift (shift->tp_, shift->t_ + t, shift->s_ + s));
            else
              return Ptr_t(new Shift (tp, t, s));
          }

          Shift (TimeParameterizationPtr_t tp, value_type t, value_type s)
          : tp_ (tp), t_ (t), s_ (s) {}

          value_type value (const value_type& t) const
          {
            return tp_->value (t + t_) + s_;
          }
          value_type derivative (const value_type& t) const
          {
            return tp_->derivative (t + t_);
          }
          value_type impl_derivativeBound (const value_type& l, const value_type& u) const
          {
            return tp_->derivativeBound(l + t_, u + t_);
          }

          TimeParameterizationPtr_t copy () const
          {
            return create (tp_->copy(), t_, s_);
          }

          TimeParameterizationPtr_t tp_;
          value_type t_;
          value_type s_;
      };
    }

    // Constructor with constraints
    Path::Path (const interval_t& interval, size_type outputSize,
		size_type outputDerivativeSize,
		const ConstraintSetPtr_t& constraints) :
      paramRange_ (interval), timeRange_ (interval), outputSize_ (outputSize),
      outputDerivativeSize_ (outputDerivativeSize), constraints_ ()
    {
      if (constraints) {
	constraints_ = HPP_STATIC_PTR_CAST (ConstraintSet,
					    constraints->copy ());
      }
    }

    // Constructor without constraints
    Path::Path (const interval_t& interval, size_type outputSize,
		size_type outputDerivativeSize) :
      paramRange_ (interval), timeRange_ (interval), outputSize_ (outputSize),
      outputDerivativeSize_ (outputDerivativeSize), constraints_ ()
    {
    }

    // Copy constructor
    Path::Path (const Path& path) :
      paramRange_ (path.paramRange_), timeRange_ (path.timeRange_),
      outputSize_ (path.outputSize_),
      outputDerivativeSize_ (path.outputDerivativeSize_), constraints_ (),
      timeParam_ ()
    {
      if (path.constraints_) {
	constraints_ = HPP_STATIC_PTR_CAST (ConstraintSet,
					    path.constraints_->copy ());
      }
      if (path.timeParam_)
        timeParam_ = path.timeParam_->copy();
    }

    Path::Path (const Path& path, const ConstraintSetPtr_t& constraints) :
      paramRange_ (path.paramRange_), timeRange_ (path.timeRange_),
      outputSize_ (path.outputSize_),
      outputDerivativeSize_ (path.outputDerivativeSize_),
      constraints_ (constraints), timeParam_ ()
    {
      assert (!path.constraints_);
      if (path.timeParam_)
        timeParam_ = path.timeParam_->copy();
    }

    // Initialization after creation
    void Path::init (const PathWkPtr_t& self)
    {
      weak_ = self;
    }

    PathPtr_t Path::extract (const interval_t& subInterval) const
        throw (projection_error)
    {
      PathPtr_t res;
      if (timeParam_) {
        interval_t paramInterval (
            timeParam_->value(subInterval.first),
            timeParam_->value(subInterval.second));
        res = this->impl_extract (paramInterval);
        // TODO Child class that reimplement impl_extract may return
        // a path whose paramRange has been shifted to 0. We must then shift
        // the time parameterization.
        value_type shift_t, shift_s;
        interval_t timeInterval;
        if (subInterval.first > subInterval.second) {
          shift_t = 0;
          shift_s = res->paramRange().first - paramInterval.second;

          if (shift_s != 0) {
            shift_t = subInterval.second;
            timeInterval.first = 0;
            timeInterval.second = subInterval.first - subInterval.second;
          } else {
            shift_t = 0;
            timeInterval = interval_t (subInterval.second, subInterval.first);
          }
        } else {
          shift_t = 0;
          shift_s = res->paramRange().first - paramInterval.first;
          if (shift_s != 0) {
            shift_t = subInterval.first;
            timeInterval.first = 0;
            timeInterval.second = subInterval.second - subInterval.first;
          } else {
            assert (res->paramRange() == paramInterval);
            shift_t = 0;
            timeInterval = subInterval;
          }
        }
        timeParameterization::Shift::createWithCheck (timeParam_,
            shift_t, shift_s);
#ifndef NDEBUG
        interval_t pr = res->paramRange();
#endif // NDEBUG
        res->timeParameterization(timeParam_->copy(), timeInterval);
        assert (pr == res->paramRange());
      } else {
        res = this->impl_extract (subInterval);
      }
      return res;
    }

    PathPtr_t Path::impl_extract (const interval_t& paramInterval) const
        throw (projection_error)
    {
      if (paramInterval == paramRange_)
	return this->copy ();
      return ExtractedPath::create (weak_.lock (), paramInterval);
    }

    PathPtr_t Path::reverse () const
    {
      interval_t interval;
      interval.first = this->timeRange_.second;
      interval.second = this->timeRange_.first;
      return this->extract (interval);
    }

    void Path::checkPath () const
    {
      if (constraints()) {
        if (!constraints()->isSatisfied (initial())) {
          hppDout (error, *constraints());
          hppDout (error, initial().transpose ());
          throw projection_error ("Initial configuration of path does not satisfy "
              "the constraints");
        }
        if (constraints() && !constraints()->isSatisfied (end())) {
          hppDout (error, *constraints());
          hppDout (error, end().transpose ());
          throw projection_error ("End configuration of path does not satisfy "
              "the constraints");
        }
      }
    }

    std::ostream& Path::print (std::ostream& os) const
    {
      os << "time in [ " << timeRange().first << ", "
        << timeRange().second << " ]";
      if (timeParam_)
        os << ", param in [ " << paramRange().first << ", "
          << paramRange().second << " ]";
      os << std::endl;
      return os;
    }
  } //   namespace core
} // namespace hpp
