// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef TEST_UTIL_HH
#define TEST_UTIL_HH

#define CONFIGURATION_VECTOR_IS_APPROX(robot, Qa, Qb, prec)            \
  BOOST_CHECK_MESSAGE(::hpp::pinocchio::isApprox(robot, Qa, Qb, prec), \
                      "check isApprox(" #Qa ", " #Qb                   \
                      ") failed "                                      \
                      "[\n"                                            \
                          << (Qa).transpose() << "\n!=\n"              \
                          << (Qb).transpose() << "\n]")

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, prec)              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, prec),            \
                      "check " #Va ".isApprox(" #Vb       \
                      ") failed "                         \
                      "[\n"                               \
                          << (Va).transpose() << "\n!=\n" \
                          << (Vb).transpose() << "\n]")

#define EIGEN_QUAT_IS_APPROX(Va, Vb, prec)       \
  {                                              \
    value_type _inv = (Va.dot(Vb) > 0) ? 1 : -1; \
    EIGEN_VECTOR_IS_APPROX(Va, _inv* Vb, prec);  \
  }

#define EIGEN_VECTOR_IS_NOT_APPROX(Va, Vb, prec)          \
  BOOST_CHECK_MESSAGE(!Va.isApprox(Vb, prec),             \
                      "check !" #Va ".isApprox(" #Vb      \
                      ") failed "                         \
                      "[\n"                               \
                          << (Va).transpose() << "\n==\n" \
                          << (Vb).transpose() << "\n]")

#define EIGEN_IS_APPROX(matrixA, matrixB, prec)               \
  BOOST_CHECK_MESSAGE(matrixA.isApprox(matrixB, prec),        \
                      "check " #matrixA ".isApprox(" #matrixB \
                      ") failed "                             \
                      "[\n"                                   \
                          << matrixA << "\n!=\n"              \
                          << matrixB << "\n]")

#define EIGEN_IS_NOT_APPROX(matrixA, matrixB, prec)            \
  BOOST_CHECK_MESSAGE(!matrixA.isApprox(matrixB, prec),        \
                      "check !" #matrixA ".isApprox(" #matrixB \
                      ") failed "                              \
                      "[\n"                                    \
                          << matrixA << "\n==\n"               \
                          << matrixB << "\n]")

#endif  // TEST_UTIL_HH
