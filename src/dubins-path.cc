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
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/core/dubins-path.hh>
#include <hpp/core/steering-method/constant-curvature.hh>
#include <pinocchio/spatial/se3.hpp>
#include "dubins.hh"

namespace hpp {
  namespace core {
    using steeringMethod::ConstantCurvaturePtr_t;
    using steeringMethod::ConstantCurvature;

    DubinsPathPtr_t DubinsPath::create (const DevicePtr_t& device,
					ConfigurationIn_t init,
					ConfigurationIn_t end,
                                        value_type extraLength,
					value_type rho,
					size_type xyId, size_type rzId,
                                        const std::vector<JointPtr_t> wheels)
    {
      DubinsPath* ptr = new DubinsPath (device, init, end, extraLength, rho,
                                        xyId, rzId, wheels);
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
                                        value_type extraLength,
					value_type rho,
					size_type xyId, size_type rzId,
                                        const std::vector<JointPtr_t> wheels,
					ConstraintSetPtr_t constraints)
    {
      DubinsPath* ptr = new DubinsPath (device, init, end, extraLength, rho,
                                        xyId, rzId, wheels, constraints);
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

    DubinsPath::DubinsPath (const DevicePtr_t& robot, ConfigurationIn_t init,
			    ConfigurationIn_t end, value_type extraLength,
                            value_type rho, size_type xyId, size_type rzId,
                            const std::vector<JointPtr_t> wheels) :
      parent_t (robot->configSize (), robot->numberDof ()),
      device_ (robot), initial_ (init), end_ (end), xyId_ (xyId), rzId_ (rzId),
      wheels_ (wheels), typeId_ (0), extraLength_ (extraLength), rho_ (rho)
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
			    ConfigurationIn_t end, value_type extraLength,
                            value_type rho, size_type xyId, size_type rzId,
                            const std::vector<JointPtr_t> wheels,
			    ConstraintSetPtr_t constraints) :
      parent_t (robot->configSize (), robot->numberDof ()),
      device_ (robot), initial_ (init), end_ (end), xyId_ (xyId), rzId_ (rzId),
      wheels_ (wheels), typeId_ (0), extraLength_ (extraLength), rho_ (rho)
    {
      this->constraints (constraints);
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
      dxyId_ (path.dxyId_), drzId_ (path.drzId_), wheels_ (path.wheels_),
      typeId_ (path.typeId_), lengths_(path.lengths_),
      extraLength_ (path.extraLength_), rho_ (path.rho_), qi_ (path.qi_)
    {
    }

    DubinsPath::DubinsPath (const DubinsPath& path,
			    const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_), xyId_ (path.xyId_),
      rzId_ (path.rzId_), wheels_ (path.wheels_), typeId_ (path.typeId_),
      lengths_(path.lengths_), extraLength_ (path.extraLength_),
      rho_ (path.rho_), qi_ (path.qi_)
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
      // Find rank of translation and rotation in velocity vectors
      // Hypothesis: degrees of freedom all belong to a planar joint or
      // xyId_ belong to a tranlation joint, rzId_ belongs to a SO2 joint.
      JointPtr_t joint (device_->getJointAtConfigRank (xyId_));
      size_type offset (xyId_ - joint->rankInConfiguration ());
      dxyId_ = joint->rankInVelocity () + offset;
      joint = device_->getJointAtConfigRank (rzId_);
      offset = rzId_ - joint->rankInConfiguration ();
      drzId_ = joint->rankInVelocity () + offset;
      // Create constant curvature segments
      std::vector <Configuration_t> q (4);
      q [0] = initial_;
      q [1].resize (device_->configSize ());
      q [2].resize (device_->configSize ());
      q [3] = end_;
      std::vector <value_type> extraL (3);
      value_type L (lengths_ [0] +lengths_ [1] +lengths_ [2]);
      for (std::size_t i=0; i<3; ++i) {
        extraL [i] = lengths_ [i] / L * extraLength_;
      }
      value_type l (lengths_ [0]);
      pinocchio::interpolate
        (device_, initial_.head (device_->configSize ()),
         end_.head (device_->configSize ()), l/L, q[1]);
      l += lengths_ [1];
      pinocchio::interpolate
        (device_, initial_.head (device_->configSize ()),
         end_.head (device_->configSize ()), l/L, q[2]);
      const int* types = DIRDATA [typeId_];
      for (std::size_t i=0; i<3; ++i) {
        value_type curvature = 0;
        switch (types [i]) {
        case L_SEG:
          curvature = 1./rho_;
          break;
        case S_SEG:
          curvature = 0;
          break;
        case R_SEG:
          curvature = -1./rho_;
          break;
        default:
          assert (false && "Invalid Dubins segment type.");
        }
        ConstantCurvaturePtr_t path (ConstantCurvature::create
                                     (device_, q [i], q [i+1],
                                      rho_ * lengths_ [i],
                                      rho_ * lengths_ [i] + extraL [i],
                                      curvature, xyId_, rzId_,
                                      device_->getJointAtConfigRank (rzId_),
                                      wheels_, ConstraintSetPtr_t ()));
        appendPath (path);
        q [i+1] = path->end ();
      }
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

    void dubins_segment_velocity (double t, vector3_t qi, vector3_t& v,
				  int type)
    {
      assert( type == L_SEG || type == S_SEG || type == R_SEG );

      if( type == L_SEG ) {
        v [0] = cos (qi[2]+t);
        v [1] = -sin(qi[2]+t);
        v [2] = 1;
      }
      else if( type == R_SEG ) {
        v [0] = cos (t - qi[2]);
        v [1] = sin (qi[2] - t);
        v [2] = -1;
      }
      else if( type == S_SEG ) {
        v [0] = cos(qi[2]);
        v [1] = sin(qi[2]);
        v [2] = 0;
      }
    }
  } //   namespace hpp-core
} // namespace hpp
