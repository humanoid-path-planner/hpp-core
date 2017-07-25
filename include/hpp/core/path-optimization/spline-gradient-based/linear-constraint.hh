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

namespace hpp {
  namespace core {
    namespace pathOptimization {
      // HPP_DEFINE_TIMECOUNTER(SGB_constraintDecomposition);

      template <int _PB, int _SO>
      struct SplineGradientBased<_PB, _SO>::LinearConstraint
      {
        typedef Eigen::JacobiSVD < matrix_t > Decomposition_t;

        LinearConstraint (size_type inputSize, size_type outputSize) :
          J (outputSize, inputSize), b (outputSize),
          dec (outputSize, inputSize, Eigen::ComputeThinU | Eigen::ComputeFullV),
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

        void decompose (bool check = false)
        {
          // HPP_START_TIMECOUNTER(SGB_constraintDecomposition);
          if (J.rows() == 0) { // No constraint
            PK = matrix_t::Identity (J.cols(), J.cols());
            // PK_linv = PK;
            xStar = vector_t::Zero (PK.rows());
            return;
          }

          dec.compute (J);

          PK.resize(J.cols(), J.cols() - dec.rank());
          xStar.resize (PK.rows());

          xStar = dec.solve (b);

          if (check) {
            // check that the constraints are feasible
            matrix_t error = J * xStar - b;
            if (!error.isZero()) {
              hppDout (warning, "Constraint not feasible: "
                  << error.norm() << '\n' << error.transpose());
            }
          }

          PK.noalias() = constraints::getV2(dec);
          // PK_linv = PK.adjoint();
          // assert((PK_linv * PK).eval().isIdentity());

          // HPP_STOP_AND_DISPLAY_TIMECOUNTER(SGB_constraintDecomposition);
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

          // QPr.Hpinv = PK_linv * QP.Hpinv * PK_linv.transpose();
          QPr.decompose();
        }

        bool reduceConstraint (const LinearConstraint& lc, LinearConstraint& lcr, bool decompose = true, bool check = true) const
        {
          lcr.J.noalias() = lc.J * PK;
          lcr.b.noalias() = lc.b - lc.J * xStar;

          // Decompose
          if (decompose) {
            lcr.decompose(check);
            return (lcr.J.rows() == 0 || lcr.dec.rank() == std::min(lcr.J.rows(), lcr.J.cols()));
          } else return true;
        }

        void computeSolution (const vector_t& v)
        {
          xSol.noalias() = xStar + PK * v;
          assert (isSatisfied(xSol));
        }

        bool isSatisfied (const vector_t& x)
        {
          return (J * x - b).isZero();
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
        Decomposition_t dec;

        // Data for vectorized input
        // Solutions are x = xStar + PK * v, v \in kernel(J)
        matrix_t PK;
        // matrix_t PK_linv;
        vector_t xStar, xSol;
      };
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_LINEAR_CONSTRAINT_HH
