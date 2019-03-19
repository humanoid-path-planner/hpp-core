//
// Copyright (c) 2019 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

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
        static SimpleShortcutPtr_t create (const Problem& problem);

        /// Optimize path
        virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);
      protected:
        SimpleShortcut (const Problem& problem);
      }; // class SimpleShortcut
      /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_SIMPLE_SHORTCUT_HH
