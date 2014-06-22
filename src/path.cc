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

#include "extracted-path.hh"

namespace hpp {
  namespace core {
    PathPtr_t Path::extract (const interval_t& subInterval) const
    {
      if (subInterval == timeRange_)
	return this->copy ();
      return ExtractedPath::create (weak_.lock (), subInterval);
    }

    PathPtr_t Path::reverse () const
    {
      interval_t interval;
      interval.first = this->timeRange_.second;
      interval.second = this->timeRange_.first;
      return this->extract (interval);
    }

  } //   namespace core
} // namespace hpp
