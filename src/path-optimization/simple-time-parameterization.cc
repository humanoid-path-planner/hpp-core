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

#include <hpp/core/path-optimization/simple-time-parameterization.hh>

#include <limits>

#include <pinocchio/multibody/model.hpp>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/time-parameterization/polynomial.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      using timeParameterization::Polynomial;

      namespace {
        TimeParameterizationPtr_t computeTimeParameterizationFirstOrder (
            const value_type& s0, const value_type& s1, const value_type& B,
            value_type& T) 
        {
          vector_t a(2);
          T = (s1 - s0) / B;
          a[0] = s0;
          a[1] = B;
          return TimeParameterizationPtr_t (new Polynomial (a));
        }

        TimeParameterizationPtr_t computeTimeParameterizationThirdOrder (
            const value_type& s0, const value_type& s1, const value_type& B,
            value_type& T) 
        {
          vector_t a(4);
          T = 3 * (s1 - s0) / B;
          a[0] = s0;
          a[1] = 0;
          a[2] = 3 * (s1 - s0) / (T * T);
          a[3] = - 2 * a[2] / (3 * T);
          return TimeParameterizationPtr_t (new Polynomial (a));
        }
      }

      SimpleTimeParameterizationPtr_t SimpleTimeParameterization::create (const Problem& problem)
      {
        SimpleTimeParameterizationPtr_t ptr (new SimpleTimeParameterization(problem));
        return ptr;
      }

      PathVectorPtr_t SimpleTimeParameterization::optimize (const PathVectorPtr_t& path)
      {
        const value_type infinity = std::numeric_limits<value_type>::infinity();

        const value_type safety = problem().getParameter("SimpleTimeParameterization/safety", (value_type)1);
        const bool velocity = problem().getParameter("SimpleTimeParameterization/velocity", false);

        // Retrieve velocity limits
        const DevicePtr_t& robot = problem().robot();
        size_type d = robot->numberDof() - robot->extraConfigSpace().dimension();
        vector_t ub ( robot->model().velocityLimit),
                 lb (-robot->model().velocityLimit),
                 cb ((ub + lb) / 2);
        assert (cb.size() == d);

        // The velocity must be in [lb, ub]
        ub = cb + safety * (ub - cb);
        lb = cb + safety * (lb - cb);

        // When ub or lb are NaN, set them to infinity.
        ub = (ub.array() == ub.array()).select(ub, infinity);
        lb = (lb.array() == lb.array()).select(lb, infinity);

        hppDout (info, "Lower velocity bound :" << lb.transpose());
        hppDout (info, "Upper velocity bound :" << ub.transpose());

        if (   ( ub.array() <= 0 ).any()
            && ( lb.array() >= 0 ).any())
          throw std::invalid_argument ("The case where zero is not an admissible velocity is not implemented.");

        PathVectorPtr_t input = PathVector::create(
            path->outputSize(), path->outputDerivativeSize());
        PathVectorPtr_t output = PathVector::create(
            path->outputSize(), path->outputDerivativeSize());
        path->flatten(input);

        vector_t v (robot->numberDof());
        vector_t v_inv (robot->numberDof());
        for (std::size_t i = 0; i < input->numberPaths(); ++i) {
          PathPtr_t p = input->pathAtRank(i);
          interval_t paramRange = p->timeRange();

          // Compute B
          p->velocityBound (v, paramRange.first, paramRange.second);
          v_inv = (v.array() == 0).select(infinity, v.cwiseInverse());
          const value_type B = std::min(
              ( ub.cwiseProduct(v_inv)).minCoeff(),
              (-lb.cwiseProduct(v_inv)).minCoeff());

          // Compute the polynom and total time
          value_type T;
          TimeParameterizationPtr_t tp;
          if (velocity)
            tp = computeTimeParameterizationThirdOrder (paramRange.first, paramRange.second, B, T);
          else
            tp = computeTimeParameterizationFirstOrder (paramRange.first, paramRange.second, B, T);

          PathPtr_t pp = p->copy();
          pp->timeParameterization (tp, interval_t (0, T));
        }
        return output;
      }

      SimpleTimeParameterization::SimpleTimeParameterization (const Problem& problem):
        PathOptimizer(problem) {}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
