//
// Copyright (c) 2017 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_STEERING_METHOD_FWD_HH
# define HPP_CORE_STEERING_METHOD_FWD_HH

namespace hpp {
  namespace core {
    namespace steeringMethod {
      HPP_PREDEF_CLASS (Straight);
      typedef shared_ptr <Straight> StraightPtr_t;
      HPP_PREDEF_CLASS (Interpolated);
      typedef shared_ptr <Interpolated> InterpolatedPtr_t;
      HPP_PREDEF_CLASS (CarLike);
      typedef shared_ptr <CarLike> CarLikePtr_t;
      HPP_PREDEF_CLASS (ConstantCurvature);
      typedef shared_ptr <ConstantCurvature> ConstantCurvaturePtr_t;
      HPP_PREDEF_CLASS (Dubins);
      typedef shared_ptr <Dubins> DubinsPtr_t;
      HPP_PREDEF_CLASS (ReedsShepp);
      typedef shared_ptr <ReedsShepp> ReedsSheppPtr_t;
      HPP_PREDEF_CLASS (Snibud);
      typedef shared_ptr <Snibud> SnibudPtr_t;
      template <int _PolynomeBasis, int _Order> class Spline;
      HPP_PREDEF_CLASS (Hermite);
      typedef shared_ptr <Hermite> HermitePtr_t;
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_STEERING_METHOD_FWD_HH

