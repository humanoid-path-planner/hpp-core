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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/discretized-path-validation.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/core/reeds-shepp-path.hh>

namespace hpp {
  namespace core {
    namespace {
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
    void ReedsSheppPath::CSC(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi)
    {
      value_type t, u, v, Lmin = length(), L;
      if (LpSpLp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(14, t, u, v);
        Lmin = L;
      }
      if (LpSpLp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(14, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpLp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(15, t, u, v);
        Lmin = L;
      }
      if (LpSpLp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(15, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(12, t, u, v);
        Lmin = L;
      }
      if (LpSpRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(12, -t, -u, -v);
        Lmin = L;
      }
      if (LpSpRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(13, t, u, v);
        Lmin = L;
      }
      if (LpSpRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(13, -t, -u, -v);
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
    void ReedsSheppPath::CCC(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi)
    {
      value_type t, u, v, Lmin = length(), L;
      if (LpRmL(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(0, t, u, v);
        Lmin = L;
      }
      if (LpRmL(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(0, -t, -u, -v);
        Lmin = L;
      }
      if (LpRmL(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(1, t, u, v);
        Lmin = L;
      }
      if (LpRmL(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(1, -t, -u, -v);
        Lmin = L;
      }

      // backwards
      vector2_t xyb; xyb << xy(0)*csPhi(0) + xy(1)*csPhi(1),
                            xy(0)*csPhi(1) - xy(1)*csPhi(0);
      // value_type xb = xy(0)*csPhi(0) + xy(1)*csPhi(1), yb = xy(0)*csPhi(1) - xy(1)*csPhi(0);
      if (LpRmL(xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(0, v, u, t);
        Lmin = L;
      }
      if (LpRmL(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(0, -v, -u, -t);
        Lmin = L;
      }
      if (LpRmL(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(1, v, u, t);
        Lmin = L;
      }
      if (LpRmL(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(1, -v, -u, -t);
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
    void ReedsSheppPath::CCCC(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi)
    {
      value_type t, u, v, Lmin = length(), L;
      if (LpRupLumRm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
      {
        setupPath(2, t, u, -u, v);
        Lmin = L;
      }
      if (LpRupLumRm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
      {
        setupPath(2, -t, -u, u, -v);
        Lmin = L;
      }
      if (LpRupLumRm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
      {
        setupPath(3, t, u, -u, v);
        Lmin = L;
      }
      if (LpRupLumRm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(3, -t, -u, u, -v);
        Lmin = L;
      }

      if (LpRumLumRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
      {
        setupPath(2, t, u, u, v);
        Lmin = L;
      }
      if (LpRumLumRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
      {
        setupPath(2, -t, -u, -u, -v);
        Lmin = L;
      }
      if (LpRumLumRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
      {
        setupPath(3, t, u, u, v);
        Lmin = L;
      }
      if (LpRumLumRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(3, -t, -u, -u, -v);
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
    void ReedsSheppPath::CCSC(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi)
    {
      value_type t, u, v, Lmin = length() - .5*pi, L;
      if (LpRmSmLm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(4, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmLm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(4, -t, .5*pi, -u, -v);
        Lmin = L;
      }
      if (LpRmSmLm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(5, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmLm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(5, -t, .5*pi, -u, -v);
        Lmin = L;
      }

      if (LpRmSmRm(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(8, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmRm(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(8, -t, .5*pi, -u, -v);
        Lmin = L;
      }
      if (LpRmSmRm(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(9, t, -.5*pi, u, v);
        Lmin = L;
      }
      if (LpRmSmRm(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(9, -t, .5*pi, -u, -v);
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
        setupPath(6, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmLm(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(6, -v, -u, .5*pi, -t);
        Lmin = L;
      }
      if (LpRmSmLm(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(7, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmLm(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
      {
        setupPath(7, -v, -u, .5*pi, -t);
        Lmin = L;
      }

      if (LpRmSmRm(xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(10, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmRm(xyb.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(10, -v, -u, .5*pi, -t);
        Lmin = L;
      }
      if (LpRmSmRm(xyb.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(11, v, u, -.5*pi, t);
        Lmin = L;
      }
      if (LpRmSmRm(-xyb, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(11, -v, -u, .5*pi, -t);
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
    void ReedsSheppPath::CCSCC(const vector2_t& xy, const vector2_t& csPhi, const value_type& phi)
    {
      value_type t, u, v, Lmin = length() - pi, L;
      if (LpRmSLmRp(xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
      {
        setupPath(16, t, -.5*pi, u, -.5*pi, v);
        Lmin = L;
      }
      if (LpRmSLmRp(xy.cwiseProduct(oneMP), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
      {
        setupPath(16, -t, .5*pi, -u, .5*pi, -v);
        Lmin = L;
      }
      if (LpRmSLmRp(xy.cwiseProduct(onePM), csPhi.cwiseProduct(onePM), -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
      {
        setupPath(17, t, -.5*pi, u, -.5*pi, v);
        Lmin = L;
      }
      if (LpRmSLmRp(-xy, csPhi, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        setupPath(17, -t, .5*pi, -u, .5*pi, -v);
    }

    ReedsSheppPathPtr_t ReedsSheppPath::create (const model::DevicePtr_t& device,
				    ConfigurationIn_t init,
				    ConfigurationIn_t end,
				    value_type rho,
                                    size_type xyId, size_type rzId)
    {
      ReedsSheppPath* ptr = new ReedsSheppPath (device, init, end, rho, xyId, rzId);
      ReedsSheppPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
      } catch (const std::exception& exc) {
	shPtr.reset ();
      }
      return shPtr;
    }

    ReedsSheppPathPtr_t ReedsSheppPath::create (const DevicePtr_t& device,
				    ConfigurationIn_t init,
				    ConfigurationIn_t end,
				    value_type rho,
                                    size_type xyId, size_type rzId,
				    ConstraintSetPtr_t constraints)
    {
      ReedsSheppPath* ptr = new ReedsSheppPath (device, init, end, rho, xyId, rzId,
				    constraints);
      ReedsSheppPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
	hppDout (info, "success");
      } catch (const std::exception& exc) {
	hppDout (info, "failure");
	shPtr.reset ();
      }
      return shPtr;
    }

    void ReedsSheppPath::init (ReedsSheppPathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
      buildReedsShepp ();
    }

    ReedsSheppPath::ReedsSheppPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type rho,
                        size_type xyId, size_type rzId
                        ) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof ()),
      device_ (device), initial_ (init), end_ (end),
      xyId_ (xyId), rzId_ (rzId), typeId_ (0), rho_ (rho)
    {
      assert (device);
      assert (rho_ > 0);
    }

    ReedsSheppPath::ReedsSheppPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type rho,
                        size_type xyId, size_type rzId,
			ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end),
      xyId_ (xyId), rzId_ (rzId), typeId_ (0), rho_ (rho)
    {
      assert (device);
      assert (rho_ > 0);
    }

    ReedsSheppPath::ReedsSheppPath (const ReedsSheppPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), xyId_ (path.xyId_), rzId_ (path.rzId_),
      wheels_ (path.wheels_),
      typeId_ (path.typeId_), lengths_(path.lengths_), rho_ (path.rho_)
    {
    }

    ReedsSheppPath::ReedsSheppPath (const ReedsSheppPath& path,
			const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_),
      xyId_ (path.xyId_), rzId_ (path.rzId_),
      wheels_ (path.wheels_),
      typeId_ (path.typeId_), lengths_(path.lengths_), rho_ (path.rho_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }

    inline value_type meanBounds(const JointPtr_t& j, const size_type& i)
    {
      return (j->upperBound(i) + j->lowerBound(i))/2;
    }

    inline value_type saturate (const value_type& v, const JointPtr_t& j, const size_type& i)
    {
      return std::min(j->upperBound(i), std::max(j->lowerBound(i), v));
    }

    void ReedsSheppPath::setWheelJoints (const JointPtr_t rz,
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

        const vector3_t radius = zt.transform (wheels_[rk].j->currentTransformation().getTranslation());
        const value_type left  = std::atan(radius[2] / (- radius[1] - rho_));
        const value_type right = std::atan(radius[2] / (- radius[1] + rho_));

        wheels_[rk].L = saturate(wheels_[rk].S + left, *_wheels, 0);
        wheels_[rk].R = saturate(wheels_[rk].S + right, *_wheels, 0);
        ++rk;
      }
    }

    void ReedsSheppPath::buildReedsShepp()
    {
      // rotate
      vector2_t xy =
        rotate(end_.segment<2>(xyId_) - initial_.segment<2>(xyId_),
          initial_.segment<2>(rzId_));
      xy /= rho_;
      vector2_t csPhi =
        rotate(end_.segment<2>(rzId_), initial_.segment<2>(rzId_));
      value_type phi = atan2(csPhi(1), csPhi(0));

      timeRange_.second = std::numeric_limits<value_type>::max();
      CSC  (xy, csPhi, phi);
      CCC  (xy, csPhi, phi);
      CCCC (xy, csPhi, phi);
      CCSC (xy, csPhi, phi);
      CCSCC(xy, csPhi, phi);
    }

    bool ReedsSheppPath::impl_compute (ConfigurationOut_t result,
				 value_type param) const
    {
      if (param <= timeRange ().first || timeRange ().second == 0) {
	result = initial_;
	return true;
      }
      if (param >= timeRange ().second) {
	result = end_;
	return true;
      }
      // Does a linear interpolation on all the joints.
      const value_type u = (timeRange ().second == 0)?0:param/timeRange ().second;
      model::interpolate (device_, initial_, end_, u, result);

      // Compute the position of the car.
      result.segment <2> (xyId_).setZero();
      value_type t = param, v,
                 phi = atan2(initial_(rzId_+1), initial_(rzId_));

      SegmentType lastType = RS_NOP;
      for (unsigned int i=0; i<5 && t>0; ++i)
      {
        if (lengths_[i] < 0) {
          v = std::max(-t, lengths_[i]);
          t += v;
        } else {
          v = std::min(t, lengths_[i]);
          t -= v;
        }

        switch(types[typeId_][i])
        {
          case RS_LEFT:
            result(xyId_+0) +=   sin(phi+v) - sin(phi);
            result(xyId_+1) += - cos(phi+v) + cos(phi);
            lastType = RS_LEFT;
            phi += v;
            break;
          case RS_RIGHT:
            result(xyId_+0) += - sin(phi-v) + sin(phi);
            result(xyId_+1) +=   cos(phi-v) - cos(phi);
            lastType = RS_RIGHT;
            phi -= v;
            break;
          case RS_STRAIGHT:
            result(xyId_+0) += v * cos(phi);
            result(xyId_+1) += v * sin(phi);
            lastType = RS_STRAIGHT;
            break;
          case RS_NOP:
            break;
        }
      }
      result.segment <2> (xyId_) *= rho_;
      result.segment <2> (xyId_) += initial_.segment<2>(xyId_);
      result(rzId_+0) = cos(phi);
      result(rzId_+1) = sin(phi);
      switch(lastType)
      {
        case RS_LEFT:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->L;
          break;
        case RS_RIGHT:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->R;
          break;
        case RS_STRAIGHT:
          for (std::vector<Wheels_t>::const_iterator _w = wheels_.begin();
              _w != wheels_.end(); ++_w)
            result(_w->j->rankInConfiguration()) = _w->S;
          break;
        case RS_NOP:
          break;
      }
      return true;
    }
  } //   namespace hpp-core
} // namespace hpp
