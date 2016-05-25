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

#ifndef HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH
# define HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH

# include "hpp/core/path-projector.hh"

namespace hpp {
  namespace core {
    namespace pathProjector {
      class HPP_CORE_DLLAPI Progressive : public PathProjector
      {
        public:
        typedef hpp::core::StraightPath StraightPath;
        typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;
          static ProgressivePtr_t create
	    (const DistancePtr_t& distance,
	     const SteeringMethodPtr_t& steeringMethod, value_type step)
          {
            return ProgressivePtr_t (new Progressive (distance, steeringMethod,
						      step));
          }

        protected:
          bool impl_apply (const PathPtr_t& path,
			   PathPtr_t& projection) const;

          Progressive (const DistancePtr_t& distance,
		       const SteeringMethodPtr_t& steeringMethod,
		       value_type step);

	  bool project (const PathPtr_t& path, PathPtr_t& proj) const;
        private:
          value_type step_;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH
