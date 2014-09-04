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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/connected-component.hh>

namespace hpp {
  namespace core {

    using model::displayConfig;

    Node::Node (const ConfigurationPtr_t& configuration) :
      configuration_ (configuration),
      connectedComponent_ (ConnectedComponent::create ())
    {
    }

    Node::Node (const ConfigurationPtr_t& configuration,
		ConnectedComponentPtr_t connectedComponent) :
      configuration_ (configuration),
      connectedComponent_ (connectedComponent)
    {
      assert (connectedComponent_);
    }

    void Node::addOutEdge (EdgePtr_t edge)
    {
      assert (edge->from () == this);
      // Check that same edge does not exist
      for (Edges_t::iterator it=outEdges_.begin (); it != outEdges_.end ();
	   it++) {
	if ((*it)->to () == edge->to ()) {
	  std::string msg
	    ("Attempt to insert an edge between two nodes already connected");
	  hppDout (error, msg.c_str ());
	  hppDout (error, "from: " << (*configuration_).transpose());
	  hppDout (error, "  to: " << (*(edge->to ()
					 ->configuration ())).transpose());
	  throw std::runtime_error (msg.c_str ());
	}
      }
      outEdges_.push_back (edge);
    }

    void Node::addInEdge (EdgePtr_t edge)
    {
      assert (edge->to () == this);
      // Check that same edge does not exist
      for (Edges_t::iterator it=inEdges_.begin (); it != inEdges_.end ();
	   it++) {
	if ((*it)->from () == edge->from ()) {
	  std::string msg
	    ("Attempt to insert an edge between two nodes already connected");
	  hppDout (error, msg.c_str ());
	  hppDout (error, "from: " << (*(edge->from ()
					 ->configuration ())).transpose());
	  hppDout (error, "  to: " << (*configuration_).transpose());
	  throw std::runtime_error (msg.c_str ());
	  throw std::runtime_error
	    ("Attempt to insert an edge between two nodes already connected");
	}
      }
      inEdges_.push_back (edge);
    }

    void Node::connectedComponent (const ConnectedComponentPtr_t& cc)
    {
      connectedComponent_ = cc;
    }

    ConnectedComponentPtr_t Node::connectedComponent () const
    {
      return connectedComponent_;
    }

    const Edges_t& Node::outEdges () const
    {
      return outEdges_;
    }

    const Edges_t& Node::inEdges () const
    {
      return inEdges_;
    }

    ConfigurationPtr_t Node::configuration () const
    {
      return configuration_;
    }

    std::ostream& Node::print (std::ostream& os) const
    {
      os << displayConfig (*configuration ()) << std::endl;
      return os;
    }
    std::ostream& operator<< (std::ostream& os, const Node& n)
    {
      return n.print (os);
    }
  } //   namespace core
} // namespace hpp
