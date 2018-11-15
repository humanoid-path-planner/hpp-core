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

#include <hpp/core/path-optimization/linear-constraint.hh>

#include <hpp/util/timer.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/util.hh>

// #define USE_SVD
#ifdef USE_SVD
# include <hpp/constraints/svd.hh>
#endif

namespace hpp {
  namespace core {
    namespace pathOptimization {
      HPP_DEFINE_TIMECOUNTER(LinearConstraint_decompose);

      LinearConstraint::~LinearConstraint ()
      {
        HPP_DISPLAY_TIMECOUNTER(LinearConstraint_decompose);
      }

      bool LinearConstraint::decompose (bool check, bool throwIfNotValid)
      {
        HPP_SCOPE_TIMECOUNTER(LinearConstraint_decompose);

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

        PK.noalias() = constraints::getV2(dec, rank);
#else // USE_SVD
        Eigen::ColPivHouseholderQR < matrix_t > qr (J.transpose());
        rank = qr.rank();

        PK.resize(J.cols(), J.cols() - rank);
        xStar.resize (PK.rows());

        vector_t rhs ((qr.colsPermutation().inverse() * b).head(rank));

        vector_t z (J.cols());
        z.head(rank).noalias() = 
          qr.matrixR().topLeftCorner(rank, rank).triangularView<Eigen::Upper>()
          .transpose().solve (rhs);
        z.tail(J.cols() - rank).setZero();
        xStar.noalias() = qr.householderQ() * z;

        PK.noalias() = qr.householderQ() * matrix_t::Identity(J.cols(), J.cols()).rightCols(J.cols() - rank);
#endif // USE_SVD

        if (check) {
          // check that the constraints are feasible
          matrix_t error = J * xStar - b;
          if (!error.isZero(1e-7)) {
            if (throwIfNotValid) {
              HPP_THROW(std::invalid_argument, "Constraints are not feasible.\nError is "
                  << setpyformat << one_line(error) << unsetpyformat);
            }
            hppDout (warning, "Constraint not feasible: "
                << error.norm() << '\n' << error.transpose());
            return false;
          }
        }
        return true;
      }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#ifdef USE_SVD
# undef USE_SVD
#endif // USE_SVD
