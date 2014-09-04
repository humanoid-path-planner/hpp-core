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

#ifndef HPP_CORE_NODE_HH
# define HPP_CORE_NODE_HH

# include <hpp/model/fwd.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// Node of a roadmap
    ///
    /// Stores a configuration.
    class HPP_CORE_DLLAPI Node {
    public:
      typedef std::list <EdgePtr_t> Edges_t;
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \note A new connected component is created. For consistency, the
      ///       new node is not registered in the connected component.
      Node (const ConfigurationPtr_t& configuration);
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \param connectedComponent connected component the node belongs to.
      Node (const ConfigurationPtr_t& configuration,
	    ConnectedComponentPtr_t connectedComponent);
      void addOutEdge (EdgePtr_t edge);
      void addInEdge (EdgePtr_t edge);
      /// Store the connected component the node belongs to
      void connectedComponent (const ConnectedComponentPtr_t& cc);
      ConnectedComponentPtr_t connectedComponent () const;
      /// Access to outEdges
      const Edges_t& outEdges () const;
      /// Access to inEdges
      const Edges_t& inEdges () const;
      ConfigurationPtr_t configuration () const;
      /// Print node in a stream
      std::ostream& print (std::ostream& os) const;
    private:
      ConfigurationPtr_t configuration_;
      Edges_t outEdges_;
      Edges_t inEdges_;
      ConnectedComponentPtr_t connectedComponent_;
    }; // class Node
    std::ostream& operator<< (std::ostream& os, const Node& n);
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_NODE_HH
