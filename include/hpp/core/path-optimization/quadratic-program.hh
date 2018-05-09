// Copyright (c) 2018, Joseph Mirabel
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH
# define HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH

#include <hpp/core/fwd.hh>

#include <hpp/core/path-optimization/linear-constraint.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_optimization
    /// \{
    namespace pathOptimization {
      /*/ Quadratic program
       *
       *  This class stores a quadratic cost defined by
       *  \f$ 0.5 * x^T H x + b^T x \f$ where \f$ (H, b) \f$ are the parameters.
       *
       *  It can then solve the two following program:
       *  \li Program subject to linear equality constraints
       *  \f{eqnarray*}{
       *  min & 0.5 * x^T H x + b^T x \\
       *      & A_0 * x = b_0
       *  \f}
       *  This is done via \ref reduced, \ref decompose and \ref solve methods
       *  \li Program subject to linear equality and inequality constraints:
       *  \f{eqnarray*}{
       *  min & 0.5 * x^T H x + b^T x \\
       *      & A_0 * x  =  b_0 \\
       *      & A_1 * x \ge b_1
       *  \f}
       *  This is done via \ref computeLLT and \ref solve methods
       *  and uses quadprog
       **/
      struct QuadraticProgram
      {
        typedef Eigen::JacobiSVD < matrix_t > Decomposition_t;
        typedef Eigen::LLT <matrix_t, Eigen::Lower> LLT_t;

        QuadraticProgram (size_type inputSize) :
          H (inputSize, inputSize), b (inputSize),
          dec (inputSize, inputSize, Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (inputSize)
        {
          H.setZero();
          b.setZero();
          bIsZero = true;
        }

        QuadraticProgram (const QuadraticProgram& QP, const LinearConstraint& lc) :
          H (lc.PK.cols(), lc.PK.cols()), b (lc.PK.cols()), bIsZero (false),
          dec (lc.PK.cols(), lc.PK.cols(), Eigen::ComputeThinU | Eigen::ComputeThinV),
          xStar (lc.PK.cols())
        {
          QP.reduced (lc, *this);
        }

        QuadraticProgram (const QuadraticProgram& QP) :
          H (QP.H), b (QP.b), bIsZero (QP.bIsZero),
          dec (QP.dec), xStar (QP.xStar)
        {}

        ~QuadraticProgram ();

        void addRows (const std::size_t& nbRows)
        {
          H.conservativeResize(H.rows() + nbRows, H.cols());
          b.conservativeResize(b.rows() + nbRows, b.cols());

          H.bottomRows(nbRows).setZero();
        }

        /// \name Program subject to linear equality constraints.
        /// \{

        /*/ Compute the problem
         *  \f{eqnarray*}{
         *  min & 0.5 * x^T H x + b^T x \\
         *      & lc.J * x = lc.b
         *  \f}
        **/
        void reduced (const LinearConstraint& lc, QuadraticProgram& QPr) const
        {
          matrix_t H_PK (H * lc.PK);
          QPr.H.noalias() = lc.PK.transpose() * H_PK;
          QPr.b.noalias() = H_PK.transpose() * lc.xStar;
          if (!bIsZero) {
            QPr.b.noalias() += lc.PK.transpose() * b;
          }
          QPr.bIsZero = false;
        }

        void decompose ();

        void solve ()
        {
          xStar.noalias() = - dec.solve(b);
        }

        /// \}

        /// \name Program subject to linear equality and inequality constraints.
        /// \{

        void computeLLT();

        /// Compute solution using quadprog
        /// \param ce equality constraints
        /// \param ci inequality constraints: \f$ ci.J * x \ge ci.b \f$
        /// \note \ref computeLLT must have been called before.
        double solve(const LinearConstraint& ce, const LinearConstraint& ci);

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
      };
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_QUADRATIC_PROGRAM_HH
