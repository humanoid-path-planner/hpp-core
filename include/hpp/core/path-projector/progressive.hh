// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH
# define HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH

# include <hpp/core/path-projector.hh>

namespace hpp {
  namespace core {
    namespace pathProjector {
      class HPP_CORE_DLLAPI Progressive : public PathProjector
      {
        public:
          typedef hpp::core::StraightPath StraightPath;
          typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;

          /// \todo See todo of pathProjector::Global::create
          static ProgressivePtr_t create (const DistancePtr_t& distance,
             const SteeringMethodPtr_t& steeringMethod, value_type step);

          static ProgressivePtr_t create (const ProblemConstPtr_t& problem,
              const value_type& step);

        protected:
          bool impl_apply (const PathPtr_t& path,
			   PathPtr_t& projection) const;

          Progressive (const DistancePtr_t& distance,
		       const SteeringMethodPtr_t& steeringMethod,
		       value_type step, value_type threshold, value_type hessianBound);

	  bool project (const PathPtr_t& path, PathPtr_t& proj) const;

        private:
          value_type step_;
          const value_type thresholdMin_;
          const value_type hessianBound_;
          const bool withHessianBound_;
      };
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_PROGRESSIVE_HH
