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

#ifndef HPP_CORE_EDGE_HH
# define HPP_CORE_EDGE_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup roadmap
    /// \{

    /// Edge of a roadmap
    ///
    /// Links two nodes and stores a path linking the configurations stored in
    /// the nodes the edge links.
    class HPP_CORE_DLLAPI Edge
    {
    public:
      Edge (NodePtr_t n1, NodePtr_t n2, const PathPtr_t& path) :
	n1_ (n1), n2_ (n2), path_ (path)
      {
      }
      NodePtr_t from () const
      {
	return n1_;
      }
      NodePtr_t to () const
      {
	return n2_;
      }
      PathPtr_t path () const
      {
	return path_;
      }
    private:
      NodePtr_t n1_;
      NodePtr_t n2_;
      PathPtr_t path_;
    }; // class Edge
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_EDGE_HH
