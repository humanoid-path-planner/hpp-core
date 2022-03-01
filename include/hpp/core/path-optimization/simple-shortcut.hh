//
// Copyright (c) 2019 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SIMPLE_SHORTCUT_HH
# define HPP_CORE_PATH_OPTIMIZATION_SIMPLE_SHORTCUT_HH

# include <hpp/core/path-optimizer.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// \addtogroup path_optimization
      /// \{
      /// Simple shortcut
      ///
      /// Find shortest path composed of direct paths between all pairs of
      /// waypoints of input path.
      ///
      /// To do so, the optimizer builds a roadmap the nodes of which are the
      /// input path waypoints and the edges of which are the collision-free
      /// output of the steering method between all pairs of nodes.
      class HPP_CORE_DLLAPI SimpleShortcut : public PathOptimizer
      {
      public:
        /// Return shared pointer to new object.
        static SimpleShortcutPtr_t create (const ProblemConstPtr_t& problem);

        /// Optimize path
        virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);
      protected:
        SimpleShortcut (const ProblemConstPtr_t& problem);
      }; // class SimpleShortcut
      /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_SIMPLE_SHORTCUT_HH
