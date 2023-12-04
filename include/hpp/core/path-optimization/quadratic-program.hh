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

#ifndef HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH
#define HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/path-optimization/linear-constraint.hh>

namespace hpp {
namespace core {
/// \addtogroup path_optimization
/// \{
namespace pathOptimization {
/** Quadratic program
 *
 *  This class stores a quadratic cost defined by
 *  \f$ \frac{1}{2} x^T H x + b^T x \f$ where \f$ (H, b) \f$ are the parameters.
 *
 *  It can then solve the two following program:
 *  \li Program subject to linear equality constraints
 *  \f{eqnarray*}{
 *  \min & \frac{1}{2} x^T H x + b^T x \\
 *      A_0 x = b_0
 *  \f}
 *  This is done via \ref reduced, \ref decompose and \ref solve methods
 *  \li Program subject to linear equality and inequality constraints:
 *  \f{eqnarray*}{
 *  \min & \frac{1}{2} x^T H x + b^T x \\
 *      A_0 x  =  b_0 \\
 *      A_1 x \ge b_1
 *  \f}
 *  This is done via \ref computeLLT and \ref solve methods
 *  and uses quadprog
 **/
struct QuadraticProgram {
  typedef Eigen::JacobiSVD<matrix_t> Decomposition_t;
  typedef Eigen::LLT<matrix_t, Eigen::Lower> LLT_t;

  /// Constructor
  /// \param inputSize dimension of the space on which the quadratic cost is
  ///                  defined,
  /// \param useProxqp whether to use proxqp instead of eiquadprog_2011 as
  ///                  the internal solver. This parameter will soon be removed
  ///                  and proxqp be the internal solver.
  QuadraticProgram(size_type inputSize, bool useProxqp = true)
      : H(inputSize, inputSize),
        b(inputSize),
        dec(inputSize, inputSize, Eigen::ComputeThinU | Eigen::ComputeThinV),
        xStar(inputSize),
        accuracy_(1e-4),
        useProxqp_(useProxqp) {
    H.setZero();
    b.setZero();
    bIsZero = true;
  }

  /// Constructor
  /// \param inputSize dimension of the space on which the quadratic cost is
  ///                  defined,
  /// \param lc        linear equality constraint,
  /// \param useProxqp whether to use proxqp instead of eiquadprog_2011 as
  ///                  the internal solver. This parameter will soon be removed
  ///                  and proxqp be the internal solver.
  QuadraticProgram(const QuadraticProgram& QP, const LinearConstraint& lc,
                   bool useProxqp = true)
      : H(lc.PK.cols(), lc.PK.cols()),
        b(lc.PK.cols()),
        bIsZero(false),
        dec(lc.PK.cols(), lc.PK.cols(),
            Eigen::ComputeThinU | Eigen::ComputeThinV),
        xStar(lc.PK.cols()),
        accuracy_(1e-4),
        useProxqp_(useProxqp) {
    QP.reduced(lc, *this);
  }

  QuadraticProgram(const QuadraticProgram& QP)
      : H(QP.H),
        b(QP.b),
        bIsZero(QP.bIsZero),
        dec(QP.dec),
        xStar(QP.xStar),
        accuracy_(QP.accuracy_),
        useProxqp_(QP.useProxqp_) {}

  ~QuadraticProgram();

  /// Set accuracy of internal QP solver
  /// \param acc the desired accuracy
  /// \note only used by proxqp
  /// The accuracy corresponds to \f$\epsilon_{abs}\f$ in \link
  /// https://inria.hal.science/hal-03683733/file/Yet_another_QP_solver_for_robotics_and_beyond.pdf
  /// this paper \endlink (Equation (2)).
  void accuracy(value_type acc) { accuracy_ = acc; }
  /// Get accuracy of internal QP solver
  /// \note only used by proxqp
  /// The accuracy corresponds to \f$\epsilon_{abs}\f$ in \link
  /// https://inria.hal.science/hal-03683733/file/Yet_another_QP_solver_for_robotics_and_beyond.pdf
  /// this paper \endlink (Equation (2)).
  value_type accuracy() const { return accuracy_; }
  void addRows(const std::size_t& nbRows) {
    H.conservativeResize(H.rows() + nbRows, H.cols());
    b.conservativeResize(b.rows() + nbRows, b.cols());

    H.bottomRows(nbRows).setZero();
  }

  /// \name Program subject to linear equality constraints.
  /// \{

  /*/ Compute the problem
   *  \f{eqnarray*}{
   *  \min & \frac{1}{2} * x^T H x + b^T x \\
   *      lc.J * x = lc.b
   *  \f}
   **/
  void reduced(const LinearConstraint& lc, QuadraticProgram& QPr) const {
    matrix_t H_PK(H * lc.PK);
    QPr.H.noalias() = lc.PK.transpose() * H_PK;
    QPr.b.noalias() = H_PK.transpose() * lc.xStar;
    if (!bIsZero) {
      QPr.b.noalias() += lc.PK.transpose() * b;
    }
    QPr.bIsZero = false;
  }

  void decompose();

  void solve() { xStar.noalias() = -dec.solve(b); }

  /// \}

  /// \name Program subject to linear equality and inequality constraints.
  /// \{

  void computeLLT();

  /// Compute solution using quadprog
  /// \param ce equality constraints
  /// \param ci inequality constraints: \f$ ci.J * x \ge ci.b \f$
  /// \param[out] ok is set to true if a solution has been found.
  /// \return the cost of the solution.
  ///
  /// \note \ref computeLLT must have been called before.
  /// \note if the problem is ill-conditioned, member xStar is left unchanged.
  double solve(const LinearConstraint& ce, const LinearConstraint& ci, bool& ok);

  /// \}

  /// \name Model
  /// \{
  matrix_t H;
  vector_t b;
  bool bIsZero;
  /// \}

  /// \name Data (for inequality constraints)
  /// \{
  LLT_t llt;
  value_type trace;
  Eigen::VectorXi activeConstraint;
  int activeSetSize;
  /// \}

  /// \name Data (for equality constraints)
  /// \{
  Decomposition_t dec;
  vector_t xStar;
  /// \}
  value_type accuracy_;
  bool useProxqp_;
};
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH
