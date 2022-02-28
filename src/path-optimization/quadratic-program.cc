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

#include <hpp/core/path-optimization/quadratic-program.hh>

#include <hpp/util/timer.hh>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <path-optimization/spline-gradient-based/eiquadprog_2011.hpp>
#pragma GCC diagnostic pop

namespace hpp {
  namespace core {
    /// \addtogroup path_optimization
    /// \{
    namespace pathOptimization {
      HPP_DEFINE_TIMECOUNTER(QuadraticProgram_decompose);
      HPP_DEFINE_TIMECOUNTER(QuadraticProgram_computeLLT);
      HPP_DEFINE_TIMECOUNTER(QuadraticProgram_solve_quadprog);

      QuadraticProgram::~QuadraticProgram ()
      {
        HPP_DISPLAY_TIMECOUNTER(QuadraticProgram_decompose);
        HPP_DISPLAY_TIMECOUNTER(QuadraticProgram_computeLLT);
        HPP_DISPLAY_TIMECOUNTER(QuadraticProgram_solve_quadprog);
      }

      void QuadraticProgram::decompose ()
      {
        HPP_SCOPE_TIMECOUNTER(QuadraticProgram_decompose);
        dec.compute(H);
        assert(dec.rank() == H.rows());
      }

      void QuadraticProgram::computeLLT()
      {
        HPP_SCOPE_TIMECOUNTER(QuadraticProgram_computeLLT);
        trace = H.trace();
        llt.compute(H);
      }

      double QuadraticProgram::solve(const LinearConstraint& ce, const LinearConstraint& ci)
      {
        if (ce.J.rows() > ce.J.cols())
          throw std::runtime_error ("The QP is over-constrained. QuadProg cannot handle it.");
        HPP_SCOPE_TIMECOUNTER(QuadraticProgram_solve_quadprog);
        // min   0.5 * x G x + g0 x
        // s.t.  CE^T x + ce0 = 0
        //       CI^T x + ci0 >= 0
        Eigen::QuadProgStatus status;
        double cost = solve_quadprog2 (llt, trace, b,
            ce.J.transpose(), - ce.b,
            ci.J.transpose(), - ci.b,
            xStar, activeConstraint, activeSetSize, status);
        switch (status) {
          case Eigen::UNBOUNDED:
            hppDout (warning, "Quadratic problem is not bounded");
          case Eigen::CONVERGED:
            break;
          case Eigen::CONSTRAINT_LINEARLY_DEPENDENT:
            hppDout (error, "Constraint of quadratic problem are linearly dependent.");
            break;
        }
        return cost;
      }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
