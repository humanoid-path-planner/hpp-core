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
