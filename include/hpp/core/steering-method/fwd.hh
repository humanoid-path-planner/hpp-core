//
// Copyright (c) 2017 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_FWD_HH
# define HPP_CORE_STEERING_METHOD_FWD_HH

namespace hpp {
  namespace core {
    namespace steeringMethod {
      HPP_PREDEF_CLASS (Straight);
      typedef boost::shared_ptr <Straight> StraightPtr_t;
      HPP_PREDEF_CLASS (Interpolated);
      typedef boost::shared_ptr <Interpolated> InterpolatedPtr_t;
      HPP_PREDEF_CLASS (CarLike);
      typedef boost::shared_ptr <CarLike> CarLikePtr_t;
      HPP_PREDEF_CLASS (ConstantCurvature);
      typedef boost::shared_ptr <ConstantCurvature> ConstantCurvaturePtr_t;
      HPP_PREDEF_CLASS (Dubins);
      typedef boost::shared_ptr <Dubins> DubinsPtr_t;
      HPP_PREDEF_CLASS (ReedsShepp);
      typedef boost::shared_ptr <ReedsShepp> ReedsSheppPtr_t;
      HPP_PREDEF_CLASS (Snibud);
      typedef boost::shared_ptr <Snibud> SnibudPtr_t;
      template <int _PolynomeBasis, int _Order> class Spline;
      HPP_PREDEF_CLASS (Hermite);
      typedef boost::shared_ptr <Hermite> HermitePtr_t;
    } // namespace steeringMethod

    /// \deprecated use steeringMethod::Straight instead
    typedef steeringMethod::Straight SteeringMethodStraight;
    typedef steeringMethod::StraightPtr_t SteeringMethodStraightPtr_t;
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_STEERING_METHOD_FWD_HH

