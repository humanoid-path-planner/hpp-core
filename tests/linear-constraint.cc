// Copyright (c) 2026, CNRS
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

#define BOOST_TEST_MODULE linear_constraint
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path-optimization/linear-constraint.hh>

using hpp::core::matrix_t;
using hpp::core::size_type;
using hpp::core::vector_t;
using hpp::core::pathOptimization::LinearConstraint;

// Rows coupling variables {0, 3, 5}, {1, 4} and {2, 6, 7} (with a redundant
// row); variable 8 is free.
BOOST_AUTO_TEST_CASE(decompose_by_component) {
  matrix_t J(matrix_t::Zero(6, 9));
  J(0, 0) = 1;
  J(0, 3) = -2;
  J(1, 3) = 1;
  J(1, 5) = 1;
  J(2, 1) = 3;
  J(2, 4) = 1;
  J(3, 2) = 1;
  J(3, 6) = 1;
  J(4, 6) = 1;
  J(4, 7) = -1;
  J.row(5) = J.row(3) + 2 * J.row(4);
  vector_t x(vector_t::LinSpaced(9, -1, 1));

  LinearConstraint lc(9, 6);
  lc.J = J.sparseView();
  lc.b = J * x;
  BOOST_REQUIRE(lc.decompose(true));

  Eigen::FullPivLU<matrix_t> lu(J);
  BOOST_CHECK_EQUAL(lc.rank, lu.rank());
  BOOST_CHECK_EQUAL(lc.PK.cols(), 9 - lu.rank());
  matrix_t PK(lc.PK);
  BOOST_CHECK((J * PK).isZero(1e-12));
  BOOST_CHECK((PK.transpose() * PK).isIdentity(1e-12));
  BOOST_CHECK((J * lc.xStar - lc.b).isZero(1e-12));

  // Solutions are xStar + PK v.
  lc.computeSolution(vector_t::Ones(PK.cols()));
  BOOST_CHECK((J * lc.xSol - lc.b).isZero(1e-12));
}

BOOST_AUTO_TEST_CASE(decompose_infeasible) {
  LinearConstraint lc(2, 2);
  matrix_t J(2, 2);
  J << 1, 1, 2, 2;
  lc.J = J.sparseView();
  lc.b << 1, 3;
  BOOST_CHECK(!lc.decompose(true));
  BOOST_CHECK_THROW(lc.decompose(true, true), std::invalid_argument);
}
