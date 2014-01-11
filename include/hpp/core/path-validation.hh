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

#ifndef HPP_CORE_PATH_VALIDATION_HH
# define HPP_CORE_PATH_VALIDATION_HH

# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// Abstraction of path validation
    ///
    /// Instances of this class compute the latest valid configuration along
    /// a path.
    class HPP_CORE_DLLAPI PathValidation
    {
    public:
      /// Compute the latest valid configuration on a given path
      ///
      /// \param path the path to check for validity,
      /// \param reverse if true check from the end,
      /// \retval the extracted valid part of the path, pointer to path if
      ///         path is valid.
      /// \return whether the whole path is valid.
      virtual bool validate (const PathPtr_t& path, bool reverse,
			     PathPtr_t& validPart) = 0;
    protected:
      PathValidation ()
      {
      }
    }; // class PathValidation
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_VALIDATION_HH
