// Copyright (C) 2009 by Florian Valenza ENSEEIHT.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_LOGGER_HH
# define HPP_CORE_LOGGER_HH

#include <string>

# include <hpp/core/fwd.hh>
# include <roboptim/trajectory/sys.hh>

class Logger
{ 
public:
   virtual ~Logger( void ) {}
   virtual void write(const std::string& msg) = 0;
};


#endif
