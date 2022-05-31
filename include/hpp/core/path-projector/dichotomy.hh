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

#ifndef HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH
#define HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH

#include <hpp/core/path-projector.hh>

#include "hpp/core/problem.hh"

namespace hpp {
namespace core {
namespace pathProjector {
class HPP_CORE_DLLAPI Dichotomy : public PathProjector {
 public:
  typedef hpp::core::StraightPath StraightPath;
  typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;
  static DichotomyPtr_t create(const DistancePtr_t& distance,
                               const SteeringMethodPtr_t& steeringMethod,
                               value_type maxPathLength) {
    return DichotomyPtr_t(
        new Dichotomy(distance, steeringMethod, maxPathLength));
  }

  static DichotomyPtr_t create(const ProblemConstPtr_t& problem,
                               value_type maxPathLength) {
    return create(problem->distance(), problem->steeringMethod(),
                  maxPathLength);
  }

 protected:
  bool impl_apply(const PathPtr_t& path, PathPtr_t& projection) const;

  Dichotomy(const DistancePtr_t& distance,
            const SteeringMethodPtr_t& steeringMethod,
            value_type maxPathLength);

  bool applyToStraightPath(const StraightPathPtr_t& path,
                           PathPtr_t& projection) const;

 private:
  value_type maxPathLength_;
};
}  // namespace pathProjector
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATHPROJECTOR_DICHOTOMY_HH
