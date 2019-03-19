// Copyright (c) 2019, Joseph Mirabel
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

#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/core/path-optimization/spline-gradient-based.hh>

namespace hpp {
  namespace core {
    class SplineGradientBasedPlugin : public ProblemSolverPlugin
    {
      public:
        SplineGradientBasedPlugin ()
          : ProblemSolverPlugin ("SplineGradientBasedPlugin", "0.0")
        {}

      protected:
        virtual bool impl_initialize (ProblemSolverPtr_t ps)
        {
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical1",pathOptimization::SplineGradientBased<path::CanonicalPolynomeBasis, 1>::create);
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical2",pathOptimization::SplineGradientBased<path::CanonicalPolynomeBasis, 2>::create);
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical3",pathOptimization::SplineGradientBased<path::CanonicalPolynomeBasis, 3>::create);
          ps->pathOptimizers.add ("SplineGradientBased_bezier1",pathOptimization::SplineGradientBased<path::BernsteinBasis, 1>::create);
          // ps->pathOptimizers.add ("SplineGradientBased_bezier2",pathOptimization::SplineGradientBased<path::BernsteinBasis, 2>::create);
          ps->pathOptimizers.add ("SplineGradientBased_bezier3",pathOptimization::SplineGradientBased<path::BernsteinBasis, 3>::create);
          return true;
        }
    };
  } // namespace core
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::core::SplineGradientBasedPlugin)
