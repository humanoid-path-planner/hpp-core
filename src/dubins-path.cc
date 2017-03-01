// Copyright (c) 2008-2014, Andrew Walker
//               2017 Florent Lamiraux
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <math.h>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/dubins-path.hh>
#include "dubins.hh"

namespace hpp {
  namespace core {
    DubinsPathPtr_t DubinsPath::create (const model::DevicePtr_t& device,
					ConfigurationIn_t init,
					ConfigurationIn_t end,
					value_type rho,
					size_type xyId, size_type rzId)
    {
      DubinsPath* ptr = new DubinsPath (device, init, end, rho, xyId, rzId);
      DubinsPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
      } catch (const std::exception& exc) {
	shPtr.reset ();
      }
      return shPtr;
    }

    DubinsPathPtr_t DubinsPath::create (const DevicePtr_t& device,
					ConfigurationIn_t init,
					ConfigurationIn_t end,
					value_type rho,
					size_type xyId, size_type rzId,
					ConstraintSetPtr_t constraints)
    {
      DubinsPath* ptr = new DubinsPath (device, init, end, rho, xyId, rzId,
					constraints);
      DubinsPathPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    inline value_type meanBounds(const JointPtr_t& j, const size_type& i)
    {
      return (j->upperBound(i) + j->lowerBound(i))/2;
    }

    inline value_type saturate (const value_type& v, const JointPtr_t& j,
				const size_type& i)
    {
      return std::min(j->upperBound(i), std::max(j->lowerBound(i), v));
    }

    void DubinsPath::setWheelJoints (const JointPtr_t rz,
				     const std::vector<JointPtr_t> wheels)
    {
      Transform3f zt (rz->currentTransformation ());
      zt.inverse();

      wheels_.resize(wheels.size());
      std::size_t rk = 0;
      for (std::vector<JointPtr_t>::const_iterator _wheels = wheels.begin();
	   _wheels != wheels.end(); ++_wheels) {
        wheels_[rk].j = *_wheels;
        wheels_[rk].S = meanBounds(wheels_[rk].j, 0);

        const vector3_t radius = zt.transform
	  (wheels_[rk].j->currentTransformation().getTranslation());
        const value_type left  = std::atan(radius[2] / (- radius[1] - rho_));
        const value_type right = std::atan(radius[2] / (- radius[1] + rho_));

        wheels_[rk].L = saturate(wheels_[rk].S + left, *_wheels, 0);
        wheels_[rk].R = saturate(wheels_[rk].S + right, *_wheels, 0);
        ++rk;
      }
    }

    DubinsPath::DubinsPath (const DevicePtr_t& robot, ConfigurationIn_t init,
			    ConfigurationIn_t end, value_type rho,
			    size_type xyId, size_type rzId) :
      parent_t (interval_t (0, 1.), robot->configSize (),
		robot->numberDof ()),
      device_ (robot), initial_ (init), end_ (end),
      xyId_ (xyId), rzId_ (rzId), typeId_ (0), rho_ (rho)
    {
      assert (robot);
      assert (rho_ > 0);
      vector3_t q0, q1;
      q0 [0] = initial_ [xyId_ + 0]; q1 [0] = end_ [xyId_ + 0];
      q0 [1] = initial_ [xyId_ + 1]; q1 [1] = end_ [xyId_ + 1];
      q0 [2] = atan2 (initial_ [rzId_ + 1], initial_ [rzId_ + 0]);
      q1 [2] = atan2 (end_ [rzId_ + 1], end_ [rzId_ + 0]);
      dubins_init (q0, q1);
    }

    DubinsPath::DubinsPath (const DevicePtr_t& robot, ConfigurationIn_t init,
			    ConfigurationIn_t end, value_type rho,
			    size_type xyId, size_type rzId,
			    ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, 1.), robot->configSize (),
		robot->numberDof (), constraints),
      device_ (robot), initial_ (init), end_ (end),
      xyId_ (xyId), rzId_ (rzId), typeId_ (0), rho_ (rho)
    {
      assert (robot);
      assert (rho_ > 0);
      vector3_t q0, q1;
      q0 [0] = initial_ [xyId_ + 0]; q1 [0] = end_ [xyId_ + 0];
      q0 [1] = initial_ [xyId_ + 1]; q1 [1] = end_ [xyId_ + 1];
      q0 [2] = atan2 (initial_ [rzId_ + 1], initial_ [rzId_ + 0]);
      q1 [2] = atan2 (end_ [rzId_ + 1], end_ [rzId_ + 0]);
      dubins_init (q0, q1);
    }

    DubinsPath::DubinsPath (const DubinsPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), xyId_ (path.xyId_), rzId_ (path.rzId_),
      dxyId_ (path.dxyId_), drzId_ (path.drzId_),
      wheels_ (path.wheels_),
      typeId_ (path.typeId_), lengths_(path.lengths_), rho_ (path.rho_),
      qi_ (path.qi_)
    {
    }

    DubinsPath::DubinsPath (const DubinsPath& path,
			    const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_),
      xyId_ (path.xyId_), rzId_ (path.rzId_),
      wheels_ (path.wheels_),
      typeId_ (path.typeId_), lengths_(path.lengths_), rho_ (path.rho_),
      qi_ (path.qi_)
    {
    }

    void DubinsPath::init (DubinsPathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
    }

    void DubinsPath::dubins_init_normalised
    (double alpha, double beta, double d)
    {
      double best_cost = INFINITY;
      int    best_word;
      int    i;

      best_word = -1;
      for( i = 0; i < 6; i++ ) {
        double params[3];
        int err = dubins_words[i](alpha, beta, d, params);
        if(err == EDUBOK) {
	  double cost = params[0] + params[1] + params[2];
	  if(cost < best_cost) {
	    best_word = i;
	    best_cost = cost;
	    lengths_[0] = params[0];
	    lengths_[1] = params[1];
	    lengths_[2] = params[2];
	    typeId_ = i;
	  }
        }
      }

      if(best_word == -1) {
	hppDout (error, "Failed to build Dubins path between " <<
		 initial_.transpose () << " and "
		 << end_.transpose () << ".");
        throw std::logic_error ("Failed to build Dubins path");
      }
      typeId_ = best_word;
      timeRange_.second = rho_ * (lengths_ [0] + lengths_ [1] + lengths_ [2]);
    }

    double fmodr( double x, double y)
    {
      return x - y*floor(x/y);
    }

    double mod2pi( double theta )
    {
      return fmodr( theta, 2 * M_PI );
    }

    void DubinsPath::dubins_init (vector3_t q0, vector3_t q1)
    {
      int i;
      double dx = q1[0] - q0[0];
      double dy = q1[1] - q0[1];
      double D = sqrt( dx * dx + dy * dy );
      double d = D / rho_;
      double theta = mod2pi(atan2( dy, dx ));
      double alpha = mod2pi(q0[2] - theta);
      double beta  = mod2pi(q1[2] - theta);
      for( i = 0; i < 3; i ++ ) {
        qi_ [i] = q0 [i];
      }
      dubins_init_normalised( alpha, beta, d);
    }

    void dubins_segment( double t, vector3_t qi, vector3_t& qt, int type)
    {
      assert( type == L_SEG || type == S_SEG || type == R_SEG );

      if( type == L_SEG ) {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
      }
      else if( type == R_SEG ) {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
      }
      else if( type == S_SEG ) {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
      }
    }

    bool DubinsPath::impl_compute (ConfigurationOut_t result,
				   value_type param) const
    {
      if (param <= timeRange ().first ||
	  timeRange ().second == timeRange ().first) {
	result = initial_;
	return true;
      }
      if (param >= timeRange ().second) {
	result = end_;
	return true;
      }
      // Does a linear interpolation on all the joints.
      const value_type u = (timeRange ().second == 0)? 0 :
	param/timeRange ().second;
      model::interpolate (device_, initial_, end_, u, result);

      // Compute the position of the car.
      result.segment <2> (xyId_).setZero();
      // tprime is the normalised variant of the parameter
      double tprime = param / rho_;

      // The computation is done in five stages.
      //
      // 1. translate the components of the initial configuration to the origin
      // 2. generate the target configuration
      // 3. transform the target configuration
      //      scale the target configuration
      //      translate the target configration back to the original starting
      //      point
      //      normalise the target configurations angular component

    // The translated initial configuration
      vector3_t qi; qi << 0, 0, qi_ [2];

      // Generate the target configuration
      const int* types = DIRDATA [typeId_];
      int type;
      double p1 = lengths_ [0];
      double p2 = lengths_ [1];
      vector3_t q1; // end-of segment 1
      vector3_t q2; // end-of segment 2
      vector3_t q;
      dubins_segment( p1,      qi,    q1, types[0] );
      dubins_segment( p2,      q1,    q2, types[1] );
      if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
	type = types [0];
      }
      else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
	type = types [1];
      }
      else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
	type = types [2];
      }

      // scale the target configuration, translate back to the original starting
      // point
      result [xyId_ + 0] = q[0] * rho_ + qi_ [0];
      result [xyId_ + 1] = q[1] * rho_ + qi_ [1];
      result [rzId_ + 0] = cos (q[2]);
      result [rzId_ + 1] = sin (q[2]);

      switch(type)
      {
        case L_SEG:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->L;
          break;
        case R_SEG:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->R;
          break;
        case S_SEG:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->S;
          break;
      default:
	assert (false && "No type in Dubins path.");
      }
      return true;

    }

    void DubinsPath::impl_derivative (vectorOut_t result, const value_type& t,
				      size_type order) const
    {
    }

  } //   namespace hpp-core
} // namespace hpp
