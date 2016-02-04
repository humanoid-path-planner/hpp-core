
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

# include "hpp/core/path-projector.hh"

namespace hpp {
  namespace core {
    namespace pathProjector {
      class HPP_CORE_DLLAPI Global : public PathProjector
      {
        public:
          typedef hpp::core::StraightPath StraightPath;
          typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;

          static GlobalPtr_t create
	    (const DistancePtr_t& distance,
	     const SteeringMethodPtr_t& steeringMethod, value_type step)
          {
            return GlobalPtr_t (new Global (distance, steeringMethod,
						      step));
          }

        protected:
          bool impl_apply (const PathPtr_t& path,
			   PathPtr_t& projection) const;

          Global (const DistancePtr_t& distance,
		       const SteeringMethodPtr_t& steeringMethod,
		       value_type step);
        private:
          value_type step_;

          const value_type alphaMin;
          const value_type alphaMax;

          typedef std::list <Configuration_t,
                  Eigen::aligned_allocator <Configuration_t> > Configs_t;
          typedef std::list <value_type> Lengths_t;
          typedef std::list <value_type> Alphas_t;
          typedef std::vector <bool> Bools_t;

          bool projectOneStep (ConfigProjector& p,
              Configs_t& q, Configs_t::iterator& last,
              Bools_t& b, Lengths_t& l, Alphas_t& alpha) const;

          /// Returns the number of new points
          size_type reinterpolate (const DevicePtr_t& robot,
              Configs_t& q, const Configs_t::iterator& last,
              Bools_t& b, Lengths_t& l, Alphas_t& alpha,
              const value_type& maxDist) const;

          bool createPath (const DevicePtr_t& robot,
              const ConstraintSetPtr_t& constraint,
              const Configs_t& q, const Configs_t::iterator& last,
              const Bools_t& b, const Lengths_t& l,
              PathPtr_t& result) const;

          bool project (const PathPtr_t& path, PathPtr_t& projection) const;

          void initialConfigList (const PathPtr_t& path,
              Configs_t& cfgs) const;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_GLOBAL_HH
