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

#include <hpp/core/path-optimization/quadratic-program.hh>

#include <hpp/util/timer.hh>

#include <path-optimization/spline-gradient-based/eiquadprog_2011.hpp>

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
        HPP_SCOPE_TIMECOUNTER(QuadraticProgram_solve_quadprog);
        // min   0.5 * x G x + g0 x
        // s.t.  CE^T x + ce0 = 0
        //       CI^T x + ci0 >= 0
        return solve_quadprog2 (llt, trace, b,
            ce.J.transpose(), - ce.b,
            ci.J.transpose(), - ci.b,
            xStar, activeConstraint, activeSetSize);
      }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
