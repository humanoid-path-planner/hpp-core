//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_CORE_PATH_OPTIMIZER_HH
# define HPP_CORE_PATH_OPTIMIZER_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// Abstraction of path optimizer
    ///
    class HPP_CORE_DLLAPI PathOptimizer
    {
    public:
      /// Get problem
      const Problem& problem () const
      {
	return problem_;
      }
      /// Optimize path
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path) const = 0;
    protected:
      PathOptimizer (const Problem& problem) : problem_ (problem)
	{
	}
    private:
      const Problem& problem_;
    }; // class PathOptimizer;
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZER_HH
