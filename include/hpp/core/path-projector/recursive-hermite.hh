// Copyright (c) 2016, LAAS-CNRS
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

#ifndef HPP_CORE_PATHPROJECTOR_RECURSIVE_HERMITE_HH
# define HPP_CORE_PATHPROJECTOR_RECURSIVE_HERMITE_HH

# include "hpp/core/fwd.hh"
# include "hpp/core/config.hh"
# include "hpp/core/path-projector.hh"

namespace hpp {
  namespace core {
    namespace pathProjector {
      /// Implements
      /// "Fast Interpolation and Time-Optimization on Implicit Contact Submanifolds"
      /// from Kris Hauser.
      class HPP_CORE_DLLAPI RecursiveHermite : public PathProjector
      {
        public:
          typedef hpp::core::path::Hermite Hermite;
          typedef hpp::core::path::HermitePtr_t HermitePtr_t;

          static RecursiveHermitePtr_t create (const DistancePtr_t& distance,
              const SteeringMethodPtr_t& steeringMethod, value_type step);

          static RecursiveHermitePtr_t create (const Problem& problem,
              const value_type& step);

        protected:
          bool impl_apply (const PathPtr_t& path,
              PathPtr_t& projection) const;

          RecursiveHermite (const DistancePtr_t& distance,
              const SteeringMethodPtr_t& steeringMethod,
              const value_type& M, const value_type& beta);

          bool project (const PathPtr_t& path, PathPtr_t& proj) const;

        private:
          bool recurse (const HermitePtr_t& path, PathVectorPtr_t& proj, const value_type& thr) const;
          value_type M_, beta_;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_RECURSIVE_HERMITE_HH
