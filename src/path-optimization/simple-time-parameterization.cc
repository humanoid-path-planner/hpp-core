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
          hppDout (info, "Time parametrization returned " << a.transpose()
              << ", " << T);
          return TimeParameterizationPtr_t (new Polynomial (a));
        }

        TimeParameterizationPtr_t computeTimeParameterizationThirdOrder (
            const value_type& s0, const value_type& s1, const value_type& B,
            value_type& T)
        {
          vector_t a(4);
          T = 3 * (s1 - s0) / (2 * B);
          a[0] = s0;
          a[1] = 0;
          a[2] = 3 * (s1 - s0) / (T * T);
          a[3] = - 2 * a[2] / (3 * T);
          hppDout (info, "Time parametrization returned " << a.transpose()
              << ", " << T);
          return TimeParameterizationPtr_t (new Polynomial (a));
        }

        TimeParameterizationPtr_t computeTimeParameterizationFifthOrder (
            const value_type& s0, const value_type& s1, const value_type& B,
            const value_type& C, value_type& T)
        {
          vector_t a(6);
          if (C > 0) {
            T = std::max(
                15 * (s1 - s0) / (8 * B)                     , // Velocity limit
                sqrt((10./std::sqrt(3))) * sqrt((s1 - s0) / C) // Acceleration limit
                );
          } else {
            T = 15 * (s1 - s0) / (8 * B);
          }

          value_type Tpow = T*T*T;
          a[0] = s0;
          a[1] = 0;
          a[2] = 0;
          a[3] =  10 * (s1 - s0) / Tpow;
          Tpow *= T;
          a[4] = -15 * (s1 - s0) / Tpow;
          Tpow *= T;
          a[5] =   6 * (s1 - s0) / Tpow;
          hppDout (info, "Time parametrization returned " << a.transpose()
              << ", " << T);
          return TimeParameterizationPtr_t (new Polynomial (a));
        }

        void checkTimeParameterization (const TimeParameterizationPtr_t tp,
            const size_type order, const interval_t sr, const value_type B, const value_type& C, const value_type T)
        {
          using std::fabs;
          const value_type thr = std::sqrt(Eigen::NumTraits<value_type>::dummy_precision());
          if (   fabs(tp->value(0) - sr.first) >= thr
              || fabs(tp->value(T) - sr.second) >= thr) {
            throw std::logic_error("Boundaries of TimeParameterization are not correct.");
          }
          if (order >= 1
              && (fabs(tp->derivative(0, 1)) > thr
              ||  fabs(tp->derivative(T, 1)) > thr
              ||  ((C <= 0) && fabs(tp->derivative(T/2, 1) - B) > thr))
             ) {
            HPP_THROW(std::logic_error,
                "Derivative of TimeParameterization are not correct:"
                << "\ntp->derivative(0, 1) = " << tp->derivative(0, 1)
                << "\ntp->derivative(T, 1) = " << tp->derivative(T, 1)
                << "\ntp->derivative(T/2, 1) - B = " << tp->derivative(T/2, 1) - B
                << "\nT = " << T
                << "\nB = " << B
                << "\nC = " << C
                );
          }
          if (order >= 2
              && (fabs(tp->derivative(0, 2)) > thr
              ||  fabs(tp->derivative(T, 2)) > thr)
             ) {
            HPP_THROW(std::logic_error,
                "Derivative of TimeParameterization are not correct:"
                << "\ntp->derivative(0, 2) = " << tp->derivative(0, 2)
                << "\ntp->derivative(T, 2) = " << tp->derivative(T, 2)
                << "\nT = " << T
                );
          }
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

        const value_type safety = problem().getParameter("SimpleTimeParameterization/safety").floatValue();
        const size_type order = problem().getParameter("SimpleTimeParameterization/order").intValue();
        const value_type maxAcc = problem().getParameter("SimpleTimeParameterization/maxAcceleration").floatValue();
        if (order <= 1 && maxAcc > 0) {
          throw std::invalid_argument ("Maximum acceleration cannot be set when order is <= to 1. Please set parameter SimpleTimeParameterization/maxAcceleration to a negative value.");
        }

        // Retrieve velocity limits
        const DevicePtr_t& robot = problem().robot();
        vector_t ub ( robot->model().velocityLimit),
                 lb (-robot->model().velocityLimit),
                 cb ((ub + lb) / 2);
        assert (cb.size() + robot->extraConfigSpace().dimension()
            == robot->numberDof());

        // The velocity must be in [lb, ub]
        ub = cb + safety * (ub - cb);
        lb = cb + safety * (lb - cb);

        // When ub or lb are NaN, set them to infinity.
        ub = (ub.array() == ub.array()).select(ub,  infinity);
        lb = (lb.array() == lb.array()).select(lb, -infinity);

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
          interval_t paramRange = p->paramRange();

          // Compute B
          p->velocityBound (v, paramRange.first, paramRange.second);
          v_inv = (v.array() == 0).select(infinity, v.cwiseInverse());
          const value_type B = std::min(
              ( ub.cwiseProduct(v_inv)).minCoeff(),
              (-lb.cwiseProduct(v_inv)).minCoeff());
          assert (B > 0);

          // Compute the polynom and total time
          value_type T;
          TimeParameterizationPtr_t tp;
          switch (order) {
            case 0:
              tp = computeTimeParameterizationFirstOrder (paramRange.first, paramRange.second, B, T);
              break;
            case 1:
              tp = computeTimeParameterizationThirdOrder (paramRange.first, paramRange.second, B, T);
              break;
            case 2:
              tp = computeTimeParameterizationFifthOrder (paramRange.first, paramRange.second, B, maxAcc, T);
              break;
            default:
              throw std::invalid_argument ("Parameter SimpleTimeParameterization/order should be in { 0, 1, 2 }");
              break;
          }

          checkTimeParameterization (tp, order, paramRange, B, maxAcc, T);

          PathPtr_t pp = p->copy();
          pp->timeParameterization (tp, interval_t (0, T));

          output->appendPath (pp);
        }
        return output;
      }

      SimpleTimeParameterization::SimpleTimeParameterization (const Problem& problem):
        PathOptimizer(problem) {}

      // ----------- Declare parameters ------------------------------------- //

      HPP_START_PARAMETER_DECLARATION(SimpleTimeParameterization)
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "SimpleTimeParameterization/safety",
            "A scaling factor for the joint bounds.",
            Parameter(1.)));
      Problem::declareParameter(ParameterDescription (Parameter::INT,
            "SimpleTimeParameterization/order",
            "The desired continuity order.",
            Parameter((size_type)0)));
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "SimpleTimeParameterization/maxAcceleration",
            "The maximum acceleration for each degree of freedom. Not considered if negative.",
            Parameter((value_type)-1)));
      HPP_END_PARAMETER_DECLARATION(SimpleTimeParameterization)
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
