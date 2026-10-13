// Copyright (c) 2018, Joseph Mirabel
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

#include <hpp/core/path-optimization/linear-constraint.hh>
#include <hpp/pinocchio/util.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>
#include <numeric>

namespace hpp {
namespace core {
namespace pathOptimization {
HPP_DEFINE_TIMECOUNTER(LinearConstraint_decompose);

LinearConstraint::~LinearConstraint() {
  HPP_DISPLAY_TIMECOUNTER(LinearConstraint_decompose);
}

bool LinearConstraint::decompose(bool check, bool throwIfNotValid) {
  HPP_SCOPE_TIMECOUNTER(LinearConstraint_decompose);
  const size_type n = J.cols();
  J.makeCompressed();

  // Group the variables coupled by a row of J.
  std::vector<size_type> root(n);
  std::iota(root.begin(), root.end(), 0);
  auto find = [&root](size_type i) {
    while (root[i] != i) i = root[i] = root[root[i]];
    return i;
  };
  for (size_type r = 0; r < J.rows(); ++r) {
    RowMajorSparseMatrix_t::InnerIterator it(J, r);
    if (!it) continue;
    const size_type first = find(it.col());
    for (++it; it; ++it) root[find(it.col())] = first;
  }
  std::vector<size_type> component(n), local(n);
  size_type nComponents = 0;
  for (size_type i = 0; i < n; ++i)
    if (find(i) == i) component[i] = nComponents++;
  std::vector<std::vector<size_type> > columns(nComponents), rows(nComponents);
  for (size_type i = 0; i < n; ++i) {
    component[i] = component[find(i)];
    local[i] = columns[component[i]].size();
    columns[component[i]].push_back(i);
  }
  for (size_type r = 0; r < J.rows(); ++r) {
    RowMajorSparseMatrix_t::InnerIterator it(J, r);
    if (it) rows[component[it.col()]].push_back(r);
  }

  // Decompose each component.
  rank = 0;
  xStar = vector_t::Zero(n);
  std::vector<Eigen::Triplet<value_type> > kernel;
  size_type kernelCol = 0;
  for (std::size_t c = 0; c < columns.size(); ++c) {
    const std::vector<size_type>& cols = columns[c];
    const size_type nc = cols.size();
    if (rows[c].empty()) {
      for (size_type i = 0; i < nc; ++i)
        kernel.emplace_back(cols[i], kernelCol++, 1);
      continue;
    }
    matrix_t Jc(matrix_t::Zero(rows[c].size(), nc));
    vector_t bc(rows[c].size());
    for (std::size_t i = 0; i < rows[c].size(); ++i) {
      for (RowMajorSparseMatrix_t::InnerIterator it(J, rows[c][i]); it; ++it)
        Jc(i, local[it.col()]) = it.value();
      bc[i] = b[rows[c][i]];
    }
    Eigen::ColPivHouseholderQR<matrix_t> qr(Jc.transpose());
    const size_type rc = qr.rank();
    rank += rc;

    vector_t rhs((qr.colsPermutation().inverse() * bc).head(rc));
    vector_t z(vector_t::Zero(nc));
    z.head(rc).noalias() = qr.matrixR()
                               .topLeftCorner(rc, rc)
                               .triangularView<Eigen::Upper>()
                               .transpose()
                               .solve(rhs);
    vector_t xc(qr.householderQ() * z);
    matrix_t PKc(qr.householderQ() *
                 matrix_t::Identity(nc, nc).rightCols(nc - rc));
    for (size_type i = 0; i < nc; ++i) {
      xStar[cols[i]] = xc[i];
      for (size_type j = 0; j < nc - rc; ++j)
        if (PKc(i, j) != 0)
          kernel.emplace_back(cols[i], kernelCol + j, PKc(i, j));
    }
    kernelCol += nc - rc;
  }
  PK.resize(n, kernelCol);
  PK.setFromTriplets(kernel.begin(), kernel.end());

  if (check) {
    // check that the constraints are feasible
    vector_t error = J * xStar - b;
    if (!error.isZero(1e-7)) {
      if (throwIfNotValid) {
        HPP_THROW(std::invalid_argument,
                  "Constraints are not feasible.\nError is "
                      << setpyformat << one_line(error) << unsetpyformat);
      }
      hppDout(warning, "Constraint not feasible: " << error.norm() << '\n'
                                                   << error.transpose());
      return false;
    }
  }
  return true;
}

void LinearConstraint::computeRank() {
  if (J.size() == 0)
    rank = 0;
  else {
    Eigen::FullPivLU<matrix_t> lu{matrix_t(J)};
    rank = lu.rank();
  }
}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
