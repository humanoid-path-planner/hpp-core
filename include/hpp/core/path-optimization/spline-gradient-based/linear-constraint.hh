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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH

#include <hpp/constraints/svd.hh>

// #define USE_SVD

namespace hpp {
  namespace core {
    namespace pathOptimization {
      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::LinearConstraint
      {
        LinearConstraint (size_type inputSize, size_type outputSize) :
          J (outputSize, inputSize), b (outputSize),
          xSol (inputSize)
        {
          J.setZero();
          b.setZero();
        }

        void concatenate (const LinearConstraint& oc)
        {
          assert(oc.J.cols() == J.cols());
          J.conservativeResize (J.rows() + oc.J.rows(), J.cols());
          J.bottomRows(oc.J.rows()) = oc.J;
          b.conservativeResize (b.rows() + oc.b.rows());
          b.tail(oc.b.rows()) = oc.b;
        }

        /// Compute one solution and a base of the kernel of matrix J.
        /// rank is also updated.
        bool decompose (bool check = false)
        {
          // HPP_START_TIMECOUNTER(SGB_constraintDecomposition);
          if (J.rows() == 0) { // No constraint
            PK = matrix_t::Identity (J.cols(), J.cols());
            xStar = vector_t::Zero (PK.rows());
            return true;
          }

#ifdef USE_SVD
          typedef Eigen::JacobiSVD < matrix_t > Decomposition_t; 
          Decomposition_t dec (J, Eigen::ComputeThinU | Eigen::ComputeFullV);
          rank = dec.rank();

          PK.resize(J.cols(), J.cols() - rank);
          xStar.resize (PK.rows());

          xStar = dec.solve (b);

          PK.noalias() = constraints::getV2(dec);
#else // USE_SVD
          Eigen::ColPivHouseholderQR < matrix_t > qr (J.transpose());
          rank = qr.rank();

          PK.resize(J.cols(), J.cols() - rank);
          xStar.resize (PK.rows());

          vector_t rhs ((qr.colsPermutation().inverse() * b).head(rank));

          vector_t z (J.cols());
          z.head(rank).noalias() = 
            qr.matrixR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>()
            .transpose().solve (rhs);
          z.tail(J.cols() - rank).setZero();
          xStar.noalias() = qr.householderQ() * z;

          PK.noalias() = qr.householderQ() * matrix_t::Identity(J.cols(), J.cols()).rightCols(J.cols() - rank);
#endif // USE_SVD

          if (check) {
            // check that the constraints are feasible
            matrix_t error = J * xStar - b;
            if (!error.isZero(1e-7)) {
              hppDout (warning, "Constraint not feasible: "
                  << error.norm() << '\n' << error.transpose());
              return false;
            }
          }
          return true;
        }

        /// Compute rank of the constraint using a LU decomposition
        void computeRank ()
        {
          if (J.size() == 0) rank = 0;
          else {
            Eigen::FullPivLU <matrix_t> lu (J);
            rank = lu.rank();
          }
        }

        void reduceProblem (const QuadraticProblem& QP, QuadraticProblem& QPr) const
        {
          matrix_t H_PK (QP.H * PK);
          QPr.H.noalias() = PK.transpose() * H_PK;
          QPr.b.noalias() = H_PK.transpose() * xStar;
          if (!QP.bIsZero) {
            QPr.b.noalias() += PK.transpose() * QP.b;
          }
          QPr.bIsZero = false;
        }

        /// Reduced constraint into the set of solutions of this constraint.
        /// \param[in]  lc the full constraint
        /// \param[out] lcr the reduced constraint
        /// \return if computeRank, returns true if the reduced constraint is full rank.
        ///         if not computeRank, returns true.
        /// \note rank is computed using computeRank method.
        bool reduceConstraint (const LinearConstraint& lc, LinearConstraint& lcr, bool computeRank = true) const
        {
          lcr.J.noalias() = lc.J * PK;
          lcr.b.noalias() = lc.b - lc.J * xStar;

          // Decompose
          if (computeRank) {
            lcr.computeRank();
            return lcr.rank == std::min(lcr.J.rows(), lcr.J.cols());
          } else return true;
        }

        /// Write the unique solution derived from v into xSol
        /// \param v an element of the kernel of matrix J.
        void computeSolution (const vector_t& v)
        {
          xSol.noalias() = xStar + PK * v;
          assert (isSatisfied(xSol));
        }

        bool isSatisfied (const vector_t& x)
        {
          vector_t err (J * x - b);
          if (err.isZero()) return true;
          hppDout (error, "constraints could not be satisfied: " << err.norm() << '\n' << err);
          return false;
        }

        void addRows (const std::size_t& nbRows)
        {
          if (nbRows > 0) {
            J.conservativeResize(J.rows() + nbRows, J.cols());
            b.conservativeResize(b.rows() + nbRows);

            J.bottomRows(nbRows).setZero();
          }
        }

        // model
        matrix_t J;
        vector_t b;

        // Data
        size_type rank;

        // Data for vectorized input
        // Solutions are x = xStar + PK * v, v \in kernel(J)
        matrix_t PK;
        // matrix_t PK_linv;
        vector_t xStar, xSol;
      };
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#ifdef USE_SVD
# undef USE_SVD
#endif // USE_SVD

#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
