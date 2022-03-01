// Copyright (c) 2016, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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

          static RecursiveHermitePtr_t create (const ProblemConstPtr_t& problem,
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
