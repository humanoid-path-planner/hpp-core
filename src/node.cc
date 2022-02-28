//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/core/node.hh>

#include <stdexcept>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/edge.hh>
#include <hpp/core/connected-component.hh>

namespace hpp {
  namespace core {

    using pinocchio::displayConfig;

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
	   ++it) {
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
	   ++it) {
	if ((*it)->from () == edge->from ()) {
	  std::string msg
	    ("Attempt to insert an edge between two nodes already connected");
	  hppDout (error, msg.c_str ());
	  hppDout (error, "from: " << (*(edge->from ()
					 ->configuration ())).transpose());
	  hppDout (error, "  to: " << (*configuration_).transpose());
	  throw std::runtime_error (msg.c_str ());
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

    bool Node::isOutNeighbor (const NodePtr_t& n) const
    {
      for (Edges_t::const_iterator it=outEdges_.begin (); it != outEdges_.end ();
	   ++it)
	if ((*it)->to () == n) return true;
      return false;
    }

    bool Node::isInNeighbor (const NodePtr_t& n) const
    {
      for (Edges_t::const_iterator it=inEdges_.begin (); it != inEdges_.end ();
	   ++it)
	if ((*it)->from () == n) return true;
      return false;
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
