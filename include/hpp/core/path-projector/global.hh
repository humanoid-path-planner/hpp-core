
// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_CORE_PATHPROJECTOR_GLOBAL_HH
# define HPP_CORE_PATHPROJECTOR_GLOBAL_HH

// lineSearch::FixedSequence
# include <hpp/constraints/solver/hierarchical-iterative.hh>

# include <hpp/core/path-projector.hh>
# include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    namespace pathProjector {
      class HPP_CORE_DLLAPI Global : public PathProjector
      {
        public:
          typedef hpp::core::StraightPath StraightPath;
          typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;

          /// \todo The parameter "PathProjectionHessianBound" and
          ///       "PathProjectionMinimalDist" are taken from the
          /// SteeringMethod::problem(). So they are accessible from Python.
          /// However, the former should be deduced from the path constraints.
          /// The latter should be passed to the constructor as an argument.
          static GlobalPtr_t create (const DistancePtr_t& distance,
	     const SteeringMethodPtr_t& steeringMethod, value_type step);

          static GlobalPtr_t create (const Problem& problem,
              const value_type& step);

        protected:
          bool impl_apply (const PathPtr_t& path,
			   PathPtr_t& projection) const;

          Global (const DistancePtr_t& distance,
		       const SteeringMethodPtr_t& steeringMethod,
		       value_type step, value_type threshold, value_type hessianBound);
        private:
          value_type step_;

          const value_type hessianBound_;
          const value_type thresholdMin_;

          typedef constraints::solver::lineSearch::FixedSequence LineSearch_t;
          struct Data {
            Configuration_t q;
            value_type length; // Length between this config and the previous one
            LineSearch_t alpha;
            std::size_t Niter;
            value_type sigma;
            bool projected;
          };

          typedef std::list <Configuration_t,
                  Eigen::aligned_allocator <Configuration_t> > Configs_t;
          typedef std::list <value_type> Lengths_t;
          typedef std::list <LineSearch_t> Alphas_t;
          typedef std::vector <bool> Bools_t;
          typedef std::list<Data> Datas_t;

          bool projectOneStep (ConfigProjector& p,
              Configs_t& q, Configs_t::iterator& last,
              Bools_t& b, Lengths_t& l, Alphas_t& alpha) const;

          bool projectOneStep (ConfigProjector& p,
              Datas_t& ds, const Datas_t::iterator& last) const;

          /// Returns the number of new points
          size_type reinterpolate (const DevicePtr_t& robot,
              Configs_t& q, const Configs_t::iterator& last,
              Bools_t& b, Lengths_t& l, Alphas_t& alpha,
              const value_type& maxDist) const;

          /// Returns the number of new points
          size_type reinterpolate (const DevicePtr_t& robot,
              ConfigProjector& p, Datas_t& q, Datas_t::iterator& last) const;

          bool createPath (const DevicePtr_t& robot,
              const ConstraintSetPtr_t& constraint,
              const Configs_t& q, const Configs_t::iterator& last,
              const Bools_t& b, const Lengths_t& l,
              PathPtr_t& result) const;

          bool createPath (const DevicePtr_t& robot,
              const ConstraintSetPtr_t& constraint,
              const Datas_t& ds, const Datas_t::iterator& last,
              PathPtr_t& result) const;

          bool project (const PathPtr_t& path, PathPtr_t& projection) const;

          bool project2 (const PathPtr_t& path, PathPtr_t& projection) const;

          void initialConfigList (const PathPtr_t& path,
              Configs_t& cfgs) const;

          void initialConfigList (const PathPtr_t& path,
              ConfigProjector& p, Datas_t& cfgs) const;

          void initData (Data& data, const Configuration_t& q,
              ConfigProjector& p, bool computeSigma = false,
              bool projected = false,
              const Configuration_t& distTo = Configuration_t()) const;

          mutable Configuration_t q_;
          mutable vector_t dq_;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_GLOBAL_HH
