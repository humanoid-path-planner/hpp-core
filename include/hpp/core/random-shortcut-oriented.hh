//
// Copyright (c) 2016 CNRS
// Authors: Pierre Fernbach
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

#ifndef HPP_CORE_RANDOM_SHORTCUT_ORIENTED_HH
#define HPP_CORE_RANDOM_SHORTCUT_ORIENTED_HH

#include <hpp/core/random-shortcut.hh>

namespace hpp {
  namespace core {
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
    class HPP_CORE_DLLAPI RandomShortcutOriented : public RandomShortcut
    {
    public:
      /// Return shared pointer to new object.
      static RandomShortcutOrientedPtr_t create (const Problem& problem);

      /// Optimize path
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);
    protected:
      RandomShortcutOriented (const Problem& problem);
    }; // class RandomShortcut
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_RANDOM_SHORTCUT_ORIENTED_HH
