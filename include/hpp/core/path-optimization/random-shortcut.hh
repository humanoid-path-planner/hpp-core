//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH
# define HPP_CORE_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH

# include <hpp/core/path-optimizer.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
    /// \addtogroup path_optimization
    /// \{

    /// Random shortcut
    ///
    /// Path optimizer that iteratively samples random configurations along a
    /// path and that tries to connect these configurations by a call to
    /// the steering method.
    ///
    /// \note The optimizer assumes that the input path is a vector of optimal
    ///       paths for the distance function.
    class HPP_CORE_DLLAPI RandomShortcut : public PathOptimizer
    {
    public:
      /// Return shared pointer to new object.
      static RandomShortcutPtr_t create (const ProblemConstPtr_t& problem);

      /// Optimize path
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);
    protected:
      RandomShortcut (const ProblemConstPtr_t& problem);

      /// Sample times along currentOpt
      /// \param currentOpt the current path
      /// \param[in] t0, t3 are the start and end of currentOpt
      /// \param[out] t1, t2  are the position where to apply shortcut.
      /// \warning The function should output t1 < t2
      /// \return true in case of success
      virtual bool shootTimes (const PathVectorPtr_t& currentOpt,
          const value_type& t0,
          value_type& t1,
          value_type& t2,
          const value_type& t3);
    }; // class RandomShortcut
    /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH
