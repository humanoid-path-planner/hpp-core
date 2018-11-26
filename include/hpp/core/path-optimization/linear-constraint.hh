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

#include <hpp/core/fwd.hh>

#include <hpp/util/debug.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// A linear constraint \f$ J \times x = b \f$
      struct LinearConstraint
      {
        LinearConstraint (size_type inputSize, size_type outputSize) :
          J (outputSize, inputSize), b (outputSize),
          xSol (inputSize)
        {
          J.setZero();
          b.setZero();
        }

        ~LinearConstraint ();

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
        /// \param check If true, checks whether the constraint is feasible.
        /// \return whether the constraint is feasible
        ///                 (alwys true when check is false)
        bool decompose (bool check = false, bool throwIfNotValid = false);

        /// Compute rank of the constraint using a LU decomposition
        void computeRank ()
        {
          if (J.size() == 0) rank = 0;
          else {
            Eigen::FullPivLU <matrix_t> lu (J);
            rank = lu.rank();
          }
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

        /// Compute the unique solution derived from v into \ref xSol.
        /// \f$ xSol \gets x^* + PK \times v \f$
        /// \param v an element of the kernel of matrix \ref J.
        /// \retval this->xSol
        void computeSolution (const vector_t& v)
        {
          xSol.noalias() = xStar + PK * v;
#ifdef HPP_DEBUG
          isSatisfied(xSol);
#endif // HPP_DEBUG
        }

        /// Returns \f$ ( J \times x - b ).isZero (threshold) \f$
        bool isSatisfied (const vector_t& x, const value_type& threshold = Eigen::NumTraits<value_type>::dummy_precision())
        {
          vector_t err (J * x - b);
          if (err.isZero(threshold)) return true;
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

        /// \name Model
        /// \{
        matrix_t J;
        vector_t b;
        /// \}

        /// \name Data
        ///       Solutions are \f$ \left\{ x^* + PK \times v, v \in \mathbb{R}^{nCols(J) - rank} \right\} \f$
        /// \{ 

        /// Rank of \ref J
        size_type rank;

        /// Projector onto \f$ kernel(J) \f$
        matrix_t PK;
        /// \f$ x^* \f$ is a particular solution.
        vector_t xStar, xSol;

        /// \}
      };
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
