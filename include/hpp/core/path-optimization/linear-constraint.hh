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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH

#include <hpp/core/fwd.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
/// A linear constraint \f$ J \times x = b \f$
struct LinearConstraint {
  LinearConstraint(size_type inputSize, size_type outputSize)
      : J(outputSize, inputSize), b(outputSize), xSol(inputSize) {
    J.setZero();
    b.setZero();
  }

  ~LinearConstraint();

  void concatenate(const LinearConstraint& oc) {
    assert(oc.J.cols() == J.cols());
    J.conservativeResize(J.rows() + oc.J.rows(), J.cols());
    J.bottomRows(oc.J.rows()) = oc.J;
    b.conservativeResize(b.rows() + oc.b.rows());
    b.tail(oc.b.rows()) = oc.b;
  }

  /// Compute one solution and a base of the kernel of matrix J.
  /// rank is also updated.
  /// \param check If true, checks whether the constraint is feasible.
  /// \return whether the constraint is feasible
  ///                 (alwys true when check is false)
  bool decompose(bool check = false, bool throwIfNotValid = false);

  /// Compute rank of the constraint using a LU decomposition
  void computeRank() {
    if (J.size() == 0)
      rank = 0;
    else {
      Eigen::FullPivLU<matrix_t> lu(J);
      rank = lu.rank();
    }
  }

  /// Reduced constraint into the set of solutions of this constraint.
  /// \param[in]  lc the full constraint
  /// \param[out] lcr the reduced constraint
  /// \return if computeRank, returns true if the reduced constraint is full
  /// rank.
  ///         if not computeRank, returns true.
  /// \note rank is computed using computeRank method.
  bool reduceConstraint(const LinearConstraint& lc, LinearConstraint& lcr,
                        bool computeRank = true) const {
    lcr.J.noalias() = lc.J * PK;
    lcr.b.noalias() = lc.b - lc.J * xStar;

    // Decompose
    if (computeRank) {
      lcr.computeRank();
      return lcr.rank == std::min(lcr.J.rows(), lcr.J.cols());
    } else
      return true;
  }

  /// Compute the unique solution derived from v into \ref xSol.
  /// \f$ xSol \gets x^* + PK \times v \f$
  /// \param v an element of the kernel of matrix \ref J.
  /// \retval this->xSol
  void computeSolution(const vector_t& v) {
    xSol.noalias() = xStar + PK * v;
#ifdef HPP_DEBUG
    isSatisfied(xSol);
#endif  // HPP_DEBUG
  }

  /// Returns \f$ ( J \times x - b ).isZero (threshold) \f$
  bool isSatisfied(const vector_t& x,
                   const value_type& threshold =
                       Eigen::NumTraits<value_type>::dummy_precision()) {
    vector_t err(J * x - b);
    if (err.isZero(threshold)) return true;
    hppDout(error, "constraints could not be satisfied: " << err.norm() << '\n'
                                                          << err);
    return false;
  }

  void addRows(const std::size_t& nbRows) {
    if (nbRows > 0) {
      J.conservativeResize(J.rows() + nbRows, J.cols());
      b.conservativeResize(b.rows() + nbRows);

      J.bottomRows(nbRows).setZero();
    }
  }

  /// \name Model
  /// \{
  matrix_t J;
  vector_t b;
  /// \}

  /// \name Data
  ///       Solutions are \f$ \left\{ x^* + PK \times v, v \in
  ///       \mathbb{R}^{nCols(J) - rank} \right\} \f$
  /// \{

  /// Rank of \ref J
  size_type rank;

  /// Projector onto \f$ kernel(J) \f$
  matrix_t PK;
  /// \f$ x^* \f$ is a particular solution.
  vector_t xStar, xSol;

  /// \}
};
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
