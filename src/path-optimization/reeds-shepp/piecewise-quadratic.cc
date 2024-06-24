//
// Copyright (c) 2019 CNRS
//
// Author: Florent Lamiraux
//

#include "path-optimization/reeds-shepp/piecewise-quadratic.hh"

namespace hpp {
namespace core {
namespace pathOptimization {
namespace reedsShepp {

  using hpp::core::interval_t;

  PiecewiseQuadraticPtr_t PiecewiseQuadratic::create
  (const value_type& initVel)
  {
    PiecewiseQuadratic* ptr (new PiecewiseQuadratic (initVel));
    PiecewiseQuadraticPtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
  }

  PiecewiseQuadraticPtr_t PiecewiseQuadratic::createCopy
  (const PiecewiseQuadraticPtr_t& other)
  {
    PiecewiseQuadratic* ptr (new PiecewiseQuadratic (*other));
    PiecewiseQuadraticPtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
  }

  hpp::core::TimeParameterizationPtr_t PiecewiseQuadratic::copy () const
  {
    return createCopy (weak_.lock ());
  }

  void PiecewiseQuadratic::init (const PiecewiseQuadraticWkPtr_t& weak)
  {
    weak_ = weak;
  }

  interval_t PiecewiseQuadratic::definitionInterval () const
  {
    return interval_t (0, times_.back ());
  }

  size_type PiecewiseQuadratic::findInterval (value_type t) const
  {
    assert (t > -sqrt (std::numeric_limits <value_type>::epsilon ()));
    assert (times_.size () >= 2);
    if (t < 0) t = 0;
    for (std::size_t i=1; i < times_.size (); ++i) {
      if (t <= times_ [i]) return i-1;
    }
    return times_.size () - 1;
  }

  value_type PiecewiseQuadratic::value (const value_type &t) const
  {
    size_type index = findInterval (t);
    std::size_t i ((std::size_t) index);
    value_type ti = times_ [i];
    return a_ [i] * (t-ti) * (t-ti) + b_ [i] * (t-ti) + c_ [i];
  }
  value_type PiecewiseQuadratic::derivative
  (const value_type &t, const size_type &order) const
  {
    size_type index = findInterval (t);
    if (index < 0) return 0;
    if ((std::size_t)index >= times_.size ()) return 0;
    std::size_t i ((std::size_t) index);
    value_type ti = times_ [i];
    if (order == 1) {
      return 2 * a_ [i] * (t-ti) + b_ [i];
    } else if (order == 2) {
      return 2 * a_ [i];
    } else {
      return 0;
    }
  }
  value_type PiecewiseQuadratic::derivativeBound
  (const value_type &low, const value_type &up) const
  {
    return std::max (fabs (derivative (low, 1)), fabs (derivative (up, 1)));
  }

  void PiecewiseQuadratic::addSegments
  (const value_type& distance, const value_type& accel,
   const value_type& decel, const value_type& maxVel,
   const value_type& targetVel)
  {
    value_type endValue = 0, endVel = initVel_, a, b, c;
    // endVel is the velocity at the end of the previous segment,
    // therefore at the beginning of this one.
    assert (maxVel >= targetVel);
    assert (maxVel >= initVel_);
    assert (accel > 0);
    assert (decel > 0);
    assert (distance > 0);
    if (distance < 1e-8) return;
    // time for reaching maximal velocity
    value_type tacc = (maxVel - initVel_) / accel;
    // time for reaching end velocity
    value_type tdec = (maxVel - targetVel) / decel;
    // distance travelled during tacc and tdec
    value_type dacc = .5 * tacc * (initVel_ + maxVel);
    value_type ddec = .5 * tdec * (targetVel + maxVel);
    // Test whether there is a constant speed segment
    if (dacc + ddec < distance) {
      // There is a constant speed segment between accel and decel
      value_type remainingDist = distance - dacc + ddec;
      value_type tint = remainingDist / maxVel;
      // Add 3 segment
      // Acceleration
      a = .5 * accel;    a_.push_back (a);
      b = initVel_;      b_.push_back (b);
      c = endValue;      c_.push_back (c);
      times_.push_back (times_.back () + tacc);
      endValue = a * tacc * tacc + b * tacc + c;
      endVel = 2 * a * tacc + b;
      // Constant speed
      a = 0;        a_.push_back (a);
      b = endVel;   b_.push_back (b);
      c = endValue; c_.push_back (c);
      times_.push_back (times_.back () + tint);
      endValue = a * tint *tint + b * tint + c;
      endVel = 2 * a * tint + b;
      // Deceleration
      a = -.5 * decel;   a_.push_back (a);
      b = endVel;        b_.push_back (b);
      c = endValue;      c_.push_back (c);
      times_.push_back (times_.back () + tdec);
      endValue = a * tdec *tdec + b * tdec + c;
      endVel = 2 * a * tdec + b;
    } else if (distance < .5*(endVel*endVel - targetVel*targetVel)
	       /decel) {
      // Distance too small to stop
      value_type decel2 = .5*(endVel*endVel - targetVel*targetVel)
	/ distance;
      tdec = (endVel - targetVel)/decel2;
      // Deceleration
      a = -.5 * decel2; a_.push_back (a);
      b = endVel;       b_.push_back (b);
      c = endValue;     c_.push_back (c);
      endValue = a * tdec * tdec + b * tdec + c;
      endVel = 2 * a * tdec + b;
      times_.push_back (times_.back () + tdec);
    } else {
      // There is no constant speed segment between accel and decel
      value_type vmax = sqrt ((decel*accel)/(decel + accel)*
			      (2*distance + endVel * endVel / accel +
			       targetVel * targetVel / decel));
      tacc = (vmax - endVel) / accel;
      tdec = (vmax - targetVel)/decel;
      assert (fabs (distance -
		    .5 * (tacc*(endVel+vmax) + tdec*(vmax+targetVel))) <
	      1e-8);
      // add 2 segments
      // Acceleration
      a = .5 * accel; a_.push_back (a);
      b = endVel;     b_.push_back (b);
      c = endValue;   c_.push_back (c);
      endValue = a * tacc * tacc + b * tacc + c;
      endVel = 2 * a * tacc + b;
      times_.push_back (times_.back () + tacc);
      // Deceleration
      a = -.5 * decel; a_.push_back (a);
      b = endVel;      b_.push_back (b);
      c = endValue;    c_.push_back (c);
      endValue = a * tdec * tdec + b * tdec + c;
      endVel = 2 * a * tdec + b;
      times_.push_back (times_.back () + tdec);
    }
  }

} // namespace reedsShepp
} // namespace pathOptimization
} // namespace core
} // namespace hpp
