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

#include <pinocchio/multibody/model.hpp>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      SimpleTimeParameterizationPtr_t SimpleTimeParameterization::create (const Problem& problem)
      {
        SimpleTimeParameterizationPtr_t ptr (new SimpleTimeParameterization(problem));
        return ptr;
      }

      PathVectorPtr_t SimpleTimeParameterization::optimize (const PathVectorPtr_t& path)
      {
        const value_type safety = problem().getParameter("SimpleTimeParameterization/safety", (value_type)1);

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

        hppDout (info, "Lower velocity bound :" << lb.transpose());
        hppDout (info, "Upper velocity bound :" << ub.transpose());

        if (   ( ub.array() <= 0 ).any()
            && ( lb.array() >= 0 ).any())
          throw std::invalid_argument ("The case where zero is not an admissible velocity is not implemented.");

        vector_t ub_inv (ub.cwiseInverse());
        vector_t lb_inv (lb.cwiseInverse());
        // When ub or lb are NaN, set them to infinity, or, equivalently,
        // set ub_inv or lb_inv to zero
        ub_inv = (ub.array() == ub.array()).select(ub_inv, 0);
        lb_inv = (lb.array() == lb.array()).select(lb_inv, 0);

        hppDout (info, "Inverse of lower velocity bound :" << lb_inv.transpose());
        hppDout (info, "Inverse of upper velocity bound :" << ub_inv.transpose());

        PathVectorPtr_t input = PathVector::create(
            path->outputSize(), path->outputDerivativeSize());
        PathVectorPtr_t output = PathVector::create(
            path->outputSize(), path->outputDerivativeSize());
        path->flatten(input);

        vector_t delta (robot->numberDof());
        for (std::size_t i = 0; i < input->numberPaths(); ++i) {
          PathPtr_t p = input->pathAtRank(i);
          StraightPathPtr_t sp = HPP_DYNAMIC_PTR_CAST(StraightPath, p);
          InterpolatedPathPtr_t ip = HPP_DYNAMIC_PTR_CAST(InterpolatedPath, p);
          if (sp) {
            pinocchio::difference <hpp::pinocchio::LieGroupTpl>
              (robot, sp->end(), sp->initial(), delta);

            // Shortest length
            value_type l = std::max (
              (delta.head(d).array() * ub_inv.array()).maxCoeff(),
              (delta.head(d).array() * lb_inv.array()).maxCoeff());

            output->appendPath (
                StraightPath::create (robot, sp->initial(), sp->end(), l, sp->constraints())
                );
          } else if (ip) {
            typedef InterpolatedPath::InterpolationPoints_t IPs_t;
            const IPs_t& ips = ip->interpolationPoints();
            value_type l = 0;
            std::vector<value_type> ts; ts.reserve(ips.size());
            for (IPs_t::const_iterator _ip2 = ips.begin(), _ip1 = _ip2++;
                _ip2 != ips.end(); ++_ip1, ++_ip2) {
              pinocchio::difference <hpp::pinocchio::LieGroupTpl>
                (robot, _ip2->second, _ip1->second, delta);
              // Shortest length
              ts.push_back(l);
              l += std::max (
                  (delta.head(d).array() * ub_inv.array()).maxCoeff(),
                  (delta.head(d).array() * lb_inv.array()).maxCoeff());
            }
            InterpolatedPathPtr_t nip = InterpolatedPath::create (robot, ip->initial(), ip->end(), l, ip->constraints());
            std::size_t T = 1;
            for (IPs_t::const_iterator _ip = (++ips.begin()); _ip != (--ips.end()); ++_ip)
              nip->insert(ts[T], _ip->second);
            output->appendPath(nip);
          } else {
            throw std::invalid_argument ("unknown type of paths.");
          }
        }
        return output;
      }

      SimpleTimeParameterization::SimpleTimeParameterization (const Problem& problem):
        PathOptimizer(problem) {}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
