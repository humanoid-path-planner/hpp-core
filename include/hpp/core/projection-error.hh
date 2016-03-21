// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_PROJECTION_ERROR_HH
#define HPP_CORE_PROJECTION_ERROR_HH

#include <exception>

namespace hpp {
  namespace core {
    struct HPP_CORE_DLLAPI projection_error : public std::exception
    {
      projection_error () : msg_ () {}

      projection_error (const std::string& msg) : msg_ (msg) {}

      projection_error (const projection_error& other) throw ()
        : std::exception (other), msg_ (other.msg_) {}

      virtual ~projection_error () throw () {};

      virtual const char* what () const throw () { return msg_.c_str (); };

      std::string msg_;
    };
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PROJECTION_ERROR_HH
