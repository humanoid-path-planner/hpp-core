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

#ifndef HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH
# define HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH

# include <hpp/core/path-projector.hh>

# include "hpp/core/problem.hh"

namespace hpp {
  namespace core {
    namespace pathProjector {
      class HPP_CORE_DLLAPI Dichotomy : public PathProjector
      {
        public:
        typedef hpp::core::StraightPath StraightPath;
        typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;
          static DichotomyPtr_t create
	    (const DistancePtr_t& distance,
	     const SteeringMethodPtr_t& steeringMethod,
	     value_type maxPathLength)
          {
            return DichotomyPtr_t (new Dichotomy (distance, steeringMethod,
						  maxPathLength));
          }

          static DichotomyPtr_t create
	    (const Problem& problem,
	     value_type maxPathLength)
          {
            return create (problem.distance(), problem.steeringMethod(), maxPathLength);
          }

        protected:
          bool impl_apply (const PathPtr_t& path,
			   PathPtr_t& projection) const;

          Dichotomy (const DistancePtr_t& distance,
		     const SteeringMethodPtr_t& steeringMethod,
		     value_type maxPathLength);

	  bool applyToStraightPath (const StraightPathPtr_t& path,
				    PathPtr_t& projection) const;

        private:
          value_type maxPathLength_;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH
