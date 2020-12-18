//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#include <hpp/core/reeds-shepp-path.hh>

#include <boost/math/constants/constants.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/serialization.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/steering-method/constant-curvature.hh>

namespace hpp {
  namespace core {
    namespace {
      using steeringMethod::ConstantCurvature;
      using steeringMethod::ConstantCurvaturePtr_t;

      struct Data
      {
	Data(const value_type& rho_) :
	  rsLength(std::numeric_limits <value_type>::infinity ()), rho(rho_) {}
	typedef Eigen::Matrix<value_type, 5, 1> Lengths_t;
	std::size_t typeId;
	Lengths_t lengths;
	value_type rsLength;
	value_type rho;
      }; // struct Data

      value_type precision (sqrt(std::numeric_limits <value_type>::epsilon ()));

      // The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

      const value_type pi = boost::math::constants::pi<value_type>();
      const value_type twopi = 2. * pi;
      const value_type RS_EPS = 1e-6;
      const value_type ZERO = 10*std::numeric_limits<value_type>::epsilon();
      const vector2_t oneMP = (vector2_t() << -1,1).finished();
      const vector2_t onePM = (vector2_t() << 1,-1).finished();

      enum SegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };
      // static const SegmentType types[18][5];
      const SegmentType types[18][5] = {
        { RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },             // 0
        { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },            // 1
        { RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP },           // 2
        { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP },           // 3
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 4
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 5
        { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },        // 6
        { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },       // 7
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 8
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 9
        { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },       // 10
        { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },        // 11
        { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },         // 12
        { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 13
        { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },          // 14
        { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },        // 15
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT },      // 16
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT }       // 17
      };

      template <typename D1, typename D2> inline vector2_t rotate
        (const Eigen::MatrixBase<D1>& U, const Eigen::MatrixBase<D2>& CS)
        {
          const D1& _U = U.derived(); const D2& _CS = CS.derived();
          return (vector2_t() <<   _U.dot(_CS),
                                 - _U(0)*_CS(1) + _U(1)*_CS(0)).finished();
        }

    inline value_type mod2pi(const value_type& x)
    {
      value_type v = fmod(x, twopi);
      if (v < -pi)     v += twopi;
      else if (v > pi) v -= twopi;
      return v;
    }
    inline value_type angle(const vector2_t& cs)
    {
      if (cs(1) < 0) return - std::acos (cs(0));
      else           return   std::acos (cs(0));
    }
    inline void polar(const value_type& x, const value_type& y, value_type &r, value_type &theta)
    {
      r = std::sqrt(x*x+y*y);
      theta = atan2(y, x);
    }
    inline void tauOmega(value_type u, value_type v, value_type xi, value_type eta, value_type phi, value_type &tau, value_type &omega)
    {
      value_type delta = mod2pi(u-v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
      value_type t1 = atan2(eta*A - xi*B, xi*A + eta*B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
      tau = (t2<0) ? mod2pi(t1+pi) : mod2pi(t1);
      omega = mod2pi(tau - u + v - phi) ;
    }

    // formula 8.1 in Reeds-Shepp paper
    inline bool LpSpLp(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      polar(xy(0) - csPhi(1), xy(1) - 1. + csPhi(0), u, t);
      if (t >= -ZERO)
      {
        v = mod2pi(phi - t);
        if (v >= -ZERO)
        {
          assert(fabs(u*cos(t) + csPhi(1) - xy(0)) < RS_EPS);
          assert(fabs(u*sin(t) - csPhi(0) + 1 - xy(1)) < RS_EPS);
          assert(fabs(mod2pi(t+v - phi)) < RS_EPS);
          return true;
        }
      }
      return false;
    }
    // formula 8.2
    inline bool LpSpRp(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type t1, u1;
      polar(xy(0) + csPhi(1), xy(1) - 1. - csPhi(0), u1, t1);
      u1 = u1*u1;
      if (u1 >= 4.)
      {
        value_type theta;
        u = sqrt(u1 - 4.);
        theta = atan2(2., u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        assert(fabs(2*sin(t) + u*cos(t) - csPhi(1) - xy(0)) < RS_EPS);
        assert(fabs(-2*cos(t) + u*sin(t) + csPhi(0) + 1 - xy(1)) < RS_EPS);
        assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
        return t>=-ZERO && v>=-ZERO;
      }
      return false;
    }

    void setupPath (Data& d, const std::size_t& typeId, double t,
		    double u=0, double v=0, double w=0, double x=0)
    {
      d.typeId = typeId;
      d.lengths(0) = t; d.lengths(1) = u; d.lengths(2) = v; d.lengths(3) = w;
      d.lengths(4) = x;
      d.rsLength = d.rho * d.lengths.lpNorm<1> ();
      hppDout(info, "lengths = " << d.lengths.transpose());
    }

    void CSC(Data& d, const vector2_t& xy, const vector2_t& csPhi,
	     const value_type& phi)
    {
      value_type t, u, v, Lmin = d.rsLength, L;
      if (LpSpLp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 14, t, u, v);
        Lmin = L;
      }
      if (LpSpLp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 14, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpLp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 15, t, u, v);
        Lmin = L;
      }
      if (LpSpLp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 15, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 12, t, u, v);
        Lmin = L;
      }
      if (LpSpRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 12, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 13, t, u, v);
        Lmin = L;
      }
      if (LpSpRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(d, 13, -t, -u, -v);
    }
    // formula 8.3 / 8.4  *** TYPO IN PAPER ***
    inline bool LpRmL(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) - csPhi(1), eta = xy(1) - 1. + csPhi(0), u1, theta;
      polar(xi, eta, u1, theta);
      if (u1 <= 4.)
      {
        u = -2.*asin(.25 * u1);
        t = mod2pi(theta + .5 * u + pi);
        v = mod2pi(phi - t + u);
        assert(fabs(2*(sin(t) - sin(t-u)) + csPhi(1) - xy(0)) < RS_EPS);
        assert(fabs(2*(-cos(t) + cos(t-u)) - csPhi(0) + 1 - xy(1)) < RS_EPS);
        assert(fabs(mod2pi(t-u+v - phi)) < RS_EPS);
        return t>=-ZERO && u<=ZERO;
      }
      return false;
    }
    void CCC(Data& d, const vector2_t& xy, const vector2_t& csPhi,
	     const value_type& phi)
    {
      value_type t, u, v, Lmin = d.rsLength, L;
      if (LpRmL(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 0, t, u, v);
        Lmin = L;
      }
      if (LpRmL(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 0, -t, -u, -v);
        Lmin = L;
      }
      if (LpRmL(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 1, t, u, v);
        Lmin = L;
      }
      if (LpRmL(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 1, -t, -u, -v);
        Lmin = L;
      }

      // backwards
      vector2_t xyb; xyb << xy(0)*csPhi(0) + xy(1)*csPhi(1),
                            xy(0)*csPhi(1) - xy(1)*csPhi(0);
      // value_type xb = xy(0)*csPhi(0) + xy(1)*csPhi(1), yb = xy(0)*csPhi(1) - xy(1)*csPhi(0);
      if (LpRmL(xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 0, v, u, t);
        Lmin = L;
      }
      if (LpRmL(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 0, -v, -u, -t);
        Lmin = L;
      }
      if (LpRmL(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 1, v, u, t);
        Lmin = L;
      }
      if (LpRmL(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(d, 1, -v, -u, -t);
    }
    // formula 8.7
    inline bool LpRupLumRm(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) + csPhi(1), eta = xy(1) - 1. - csPhi(0), rho = .25 * (2. + sqrt(xi*xi + eta*eta));
      if (rho <= 1.)
      {
        u = acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        assert(fabs(2*(sin(t)-sin(t-u)+sin(t-2*u))-csPhi(1) - xy(0)) < RS_EPS);
        assert(fabs(2*(-cos(t)+cos(t-u)-cos(t-2*u))+csPhi(0)+1 - xy(1)) < RS_EPS);
        assert(fabs(mod2pi(t-2*u-v - phi)) < RS_EPS);
        return t>=-ZERO && v<=ZERO;
      }
      return false;
    }
    // formula 8.8
    inline bool LpRumLumRp(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) + csPhi(1), eta = xy(1) - 1. - csPhi(0), rho = (20. - xi*xi - eta*eta) / 16.;
      if (rho>=0 && rho<=1)
      {
        u = -acos(rho);
        if (u >= -.5 * pi)
        {
          tauOmega(u, u, xi, eta, phi, t, v);
          assert(fabs(4*sin(t)-2*sin(t-u)-csPhi(1) - xy(0)) < RS_EPS);
          assert(fabs(-4*cos(t)+2*cos(t-u)+csPhi(0)+1 - xy(1)) < RS_EPS);
          assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
          return t>=-ZERO && v>=-ZERO;
        }
      }
      return false;
    }

    void CCCC(Data& d, const vector2_t& xy, const vector2_t& csPhi,
	      const value_type& phi)
    {
      value_type t, u, v, Lmin = d.rsLength, L;
      if (LpRupLumRm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
      {
        setupPath(d, 2, t, u, -u, v);
        Lmin = L;
      }
      if (LpRupLumRm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 2, -t, -u, u, -v);
        Lmin = L;
      }
      if (LpRupLumRm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 3, t, u, -u, v);
        Lmin = L;
      }
      if (LpRupLumRm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 3, -t, -u, u, -v);
        Lmin = L;
      }

      if (LpRumLumRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
      {
        setupPath(d, 2, t, u, u, v);
        Lmin = L;
      }
      if (LpRumLumRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 2, -t, -u, -u, -v);
        Lmin = L;
      }
      if (LpRumLumRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 3, t, u, u, v);
        Lmin = L;
      }
      if (LpRumLumRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(d, 3, -t, -u, -u, -v);
    }
    // formula 8.9
    inline bool LpRmSmLm(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) - csPhi(1), eta = xy(1) - 1. + csPhi(0), rho, theta;
      polar(xi, eta, rho, theta);
      if (rho >= 2.)
      {
        value_type r = sqrt(rho*rho - 4.);
        u = 2. - r;
        t = mod2pi(theta + atan2(r, -2.));
        v = mod2pi(phi - .5*pi - t);
        assert(fabs(2*(sin(t)-cos(t))-u*sin(t)+csPhi(1) - xy(0)) < RS_EPS);
        assert(fabs(-2*(sin(t)+cos(t))+u*cos(t)-csPhi(0)+1 - xy(1)) < RS_EPS);
        assert(fabs(mod2pi(t+pi/2+v-phi)) < RS_EPS);
        return t>=-ZERO && u<=ZERO && v<=ZERO;
      }
      return false;
    }
    // formula 8.10
    inline bool LpRmSmRm(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) + csPhi(1), eta = xy(1) - 1. - csPhi(0), rho, theta;
      polar(-eta, xi, rho, theta);
      if (rho >= 2.)
      {
        t = theta;
        u = 2. - rho;
        v = mod2pi(t + .5*pi - phi);
        assert(fabs(2*sin(t)-cos(t-v)-u*sin(t) - xy(0)) < RS_EPS);
        assert(fabs(-2*cos(t)-sin(t-v)+u*cos(t)+1 - xy(1)) < RS_EPS);
        assert(fabs(mod2pi(t+pi/2-v-phi)) < RS_EPS);
        return t>=-ZERO && u<=ZERO && v<=ZERO;
      }
      return false;
    }
    void CCSC(Data& d, const vector2_t& xy, const vector2_t& csPhi,
	      const value_type& phi)
    {
      value_type t, u, v, Lmin = d.rsLength - .5*pi, L;
      if (LpRmSmLm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 4, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmLm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 4, -t, .5*pi, -u, -v);
        Lmin = L;
      }
      if (LpRmSmLm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 5, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmLm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 5, -t, .5*pi, -u, -v);
        Lmin = L;
      }

      if (LpRmSmRm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 8, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmRm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 8, -t, .5*pi, -u, -v);
        Lmin = L;
      }
      if (LpRmSmRm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 9, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmRm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 9, -t, .5*pi, -u, -v);
        Lmin = L;
      }

      // backwards
      vector2_t xyb; xyb << xy(0)*csPhi(0) + xy(1)*csPhi(1),
                            xy(0)*csPhi(1) - xy(1)*csPhi(0);
      // std::cout << xyb << std::endl;
      // value_type xb = xy(0)*csPhi(0) + xy(1)*csPhi(1), yb = xy(0)*csPhi(1) - xy(1)*csPhi(0);
      // std::cout << xy(0)*csPhi(0) + xy(1)*csPhi(1) << " " << xy(0)*csPhi(1) - xy(1)*csPhi(0) << std::endl;
      if (LpRmSmLm(xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 6, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmLm(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 6, -v, -u, .5*pi, -t);
        Lmin = L;
      }
      if (LpRmSmLm(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 7, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmLm(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(d, 7, -v, -u, .5*pi, -t);
        Lmin = L;
      }

      if (LpRmSmRm(xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 10, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmRm(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 10, -v, -u, .5*pi, -t);
        Lmin = L;
      }
      if (LpRmSmRm(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 11, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmRm(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(d, 11, -v, -u, .5*pi, -t);
    }
    // formula 8.11 *** TYPO IN PAPER ***
    inline bool LpRmSLmRp(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi, value_type &t, value_type &u, value_type &v)
    {
      value_type xi = xy(0) + csPhi(1), eta = xy(1) - 1. - csPhi(0), rho, theta;
      polar(xi, eta, rho, theta);
      if (rho >= 2.)
      {
        u = 4. - sqrt(rho*rho - 4.);
        if (u <= ZERO)
        {
          t = mod2pi(atan2((4-u)*xi -2*eta, -2*xi + (u-4)*eta));
          v = mod2pi(t - phi);
          assert(fabs(4*sin(t)-2*cos(t)-u*sin(t)-csPhi(1) - xy(0)) < RS_EPS);
          assert(fabs(-4*cos(t)-2*sin(t)+u*cos(t)+csPhi(0)+1 - xy(1)) < RS_EPS);
          assert(fabs(mod2pi(t-v-phi)) < RS_EPS);
          return t>=-ZERO && v>=-ZERO;
        }
      }
      return false;
    }
    void CCSCC(Data& d, const vector2_t& xy, const vector2_t& csPhi,
	       const value_type& phi)
    {
      value_type t, u, v, Lmin = d.rsLength - pi, L;
      if (LpRmSLmRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(d, 16, t, -.5*pi, u, -.5*pi, v);
        Lmin = L;
      }
      if (LpRmSLmRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(d, 16, -t, .5*pi, -u, .5*pi, -v);
        Lmin = L;
      }
      if (LpRmSLmRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(d, 17, t, -.5*pi, u, -.5*pi, v);
        Lmin = L;
      }
      if (LpRmSLmRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(d, 17, -t, .5*pi, -u, .5*pi, -v);
    }

    inline value_type meanBounds(const JointPtr_t& j, const size_type& i)
    {
      return (j->upperBound(i) + j->lowerBound(i))/2;
    }

    inline value_type saturate (const value_type& v, const JointPtr_t& j, const size_type& i)
    {
      return std::min(j->upperBound(i), std::max(j->lowerBound(i), v));
    }

    } // namespace
    namespace steeringMethod
    {
      PathVectorPtr_t reedsSheppPathOrDistance(const DevicePtr_t& device,
        ConfigurationIn_t init, ConfigurationIn_t end,
	value_type extraLength, value_type rho, size_type xyId, size_type rzId,
	const std::vector<JointPtr_t> wheels, ConstraintSetPtr_t constraints,
	bool computeDistance, value_type& distance)
      {
	Data d(rho);
	PathVectorPtr_t res;
	distance = 0;
	if (!computeDistance)
	{
	  res = PathVector::create(device->configSize (), device->numberDof (),
				   constraints);
	}
	// Find rank of translation and rotation in velocity vectors
	// Hypothesis: degrees of freedom all belong to a planar joint or
	// xyId_ belong to a tranlation joint, rzId belongs to a SO2 joint.
	JointPtr_t xy (device->getJointAtConfigRank (xyId));
	JointPtr_t rz (device->getJointAtConfigRank (rzId));
	// rotate
	vector2_t XY =
	  rotate(end.segment<2>(xyId) - init.segment<2>(xyId),
		 init.segment<2>(rzId));
	XY /= d.rho;
	vector2_t csPhi =
	  rotate(end.segment<2>(rzId), init.segment<2>(rzId));
	value_type phi = atan2(csPhi(1), csPhi(0));

	Configuration_t qInit (init), qEnd (device->configSize ());

	if (XY.squaredNorm () + phi*phi < 1e-8) {
	  if (computeDistance)
	  {
	    distance = extraLength;
	  } else
	  {
	    ConstantCurvaturePtr_t segment
	      (ConstantCurvature::create (device, qInit, end, 0, extraLength, 0,
					  xyId, rzId, rz, wheels,
					  ConstraintSetPtr_t ()));
	    res->appendPath (segment);
	  }
	  return res;
	}
	CSC  (d, XY, csPhi, phi);
	CCC  (d, XY, csPhi, phi);
	CCCC (d, XY, csPhi, phi);
	CCSC (d, XY, csPhi, phi);
	CCSCC(d, XY, csPhi, phi);
	// build path vector
	value_type L (d.rsLength), s (0.);
	for (unsigned int i=0; i<5; ++i) {
	  if (fabs (d.lengths [i]) > precision) {
	    value_type l = d.rho * fabs (d.lengths [i]);
	    s += l;
	    if (types [d.typeId][i] == RS_NOP) break;
	    value_type curvature;
	    switch (types [d.typeId][i]) {
	    case RS_LEFT:
	      curvature = 1./d.rho;
	      break;
	    case RS_RIGHT:
	      curvature = -1./d.rho;
	      break;
	    case RS_STRAIGHT:
	      curvature = 0;
	      break;
	    case RS_NOP:
	    default:
	      abort ();
	    }
	    pinocchio::interpolate (device, init, end, s/L, qEnd);
	    if (!computeDistance)
	    {
	      ConstantCurvaturePtr_t segment
		(ConstantCurvature::create (device, qInit, qEnd,
					    d.rho * d.lengths[i],
					    l * (1 + extraLength / L),
					    curvature, xyId, rzId, rz, wheels,
					    ConstraintSetPtr_t ()));
	      res->appendPath (segment);
	      qInit = segment->end ();
	    }
	  }
	}
	if (computeDistance)
	{
	  distance = d.rsLength + extraLength;
	}
	else
	{
	  assert(res->numberPaths() > 0);
	}
	return res;
      }
    } // namespace steeringMethod
  } //   namespace hpp-core
} // namespace hpp
