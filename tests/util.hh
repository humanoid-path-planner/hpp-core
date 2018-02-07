// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#ifndef TEST_UTIL_HH
# define TEST_UTIL_HH

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, prec)                                   \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, prec),                                 \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va).transpose() << "\n!=\n" << (Vb).transpose() << "\n]")

#define EIGEN_VECTOR_IS_NOT_APPROX(Va, Vb, prec)                               \
  BOOST_CHECK_MESSAGE(!Va.isApprox(Vb, prec),                                  \
      "check !" #Va ".isApprox(" #Vb ") failed "                               \
      "[\n" << (Va).transpose() << "\n==\n" << (Vb).transpose() << "\n]")

#define EIGEN_IS_APPROX(matrixA, matrixB, prec)                                \
  BOOST_CHECK_MESSAGE(matrixA.isApprox(matrixB, prec),                         \
      "check " #matrixA ".isApprox(" #matrixB ") failed "                      \
      "[\n" << matrixA << "\n!=\n" << matrixB << "\n]")

#define EIGEN_IS_NOT_APPROX(matrixA, matrixB, prec)                            \
  BOOST_CHECK_MESSAGE(!matrixA.isApprox(matrixB, prec),                        \
      "check !" #matrixA ".isApprox(" #matrixB ") failed "                     \
      "[\n" << matrixA << "\n==\n" << matrixB << "\n]")

#endif // TEST_UTIL_HH
