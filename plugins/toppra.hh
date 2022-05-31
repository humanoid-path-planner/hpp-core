// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//          Olivier Roussel (olivier.roussel@laas.fr)
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH
#define HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH

#include <hpp/core/path-optimizer.hh>

namespace hpp {
namespace core {
namespace pathOptimization {

class TOPPRA;
typedef shared_ptr<TOPPRA> TOPPRAPtr_t;

class TOPPRA : public PathOptimizer {
 public:
  static TOPPRAPtr_t create(const ProblemConstPtr_t &p) {
    return TOPPRAPtr_t(new TOPPRA(p));
  }

  PathVectorPtr_t optimize(const PathVectorPtr_t &path);

 protected:
  using PathOptimizer::PathOptimizer;
};  // class TOPPRA

}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH
