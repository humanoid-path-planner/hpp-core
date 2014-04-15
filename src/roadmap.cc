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

#include <algorithm>
#include <hpp/util/debug.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include "nearest-neighbor.hh"
#include <hpp/core/k-d-tree.hh>

namespace hpp {
  namespace core {

    std::string displayConfig (ConfigurationIn_t q)
    {
      std::ostringstream oss;
      for (size_type i=0; i < q.size (); ++i) {
	oss << q [i] << ",";
      }
      return oss.str ();
    }

    RoadmapPtr_t Roadmap::create (const DistancePtr_t& distance, const DevicePtr_t& robot)
    {
      Roadmap* ptr = new Roadmap (distance, robot);
      return RoadmapPtr_t (ptr);
    }

    Roadmap::Roadmap (const DistancePtr_t& distance, const DevicePtr_t& robot) :
      distance_ (distance), connectedComponents_ (), nodes_ (), edges_ (),
      initNode_ (), goalNodes_ (),
      nearestNeighbor_ (),
      kdTree_(robot, distance, 4)
    {
    }

    Roadmap::~Roadmap ()
    {
      clear ();
    }

    void Roadmap::clear ()
    {
      connectedComponents_.clear ();

      for (Nodes_t::iterator it = nodes_.begin (); it != nodes_.end (); it++) {
	delete *it;
      }
      nodes_.clear ();

      for (Edges_t::iterator it = edges_.begin (); it != edges_.end (); it++) {
	delete *it;
      }
      edges_.clear ();

      goalNodes_.clear ();
      initNode_ = 0x0;
      nearestNeighbor_.clear ();
      kdTree_.clear();
    }

    NodePtr_t Roadmap::addNode (const ConfigurationPtr_t& configuration,
				ConnectedComponentPtr_t connectedComponent)
    {
      value_type distance;
      if (nodes_.size () != 0) {
	NodePtr_t nearest = nearestNode (configuration, connectedComponent,
					 distance);
	if (*(nearest->configuration ()) == *configuration) {
	  return nearest;
	}
	if (distance < 1e-4) {
	  throw std::runtime_error ("distance to nearest node too small");
	}
      }
      NodePtr_t node = new Node (configuration, connectedComponent);
      hppDout (info, "Added node: " << displayConfig (*configuration));
      nodes_.push_back (node);
      if (!connectedComponent) {
	// If connectedComponent is nil pointer, node constructor creates a new
	// connected component. This new connected component needs to be added
	// in the roadmap and the new node needs to be registered in the
	// connected component.
	addConnectedComponent (node);
      } else {
	// Otherwise the new node needs to be registered in the connected
	// component. Note
	connectedComponent->addNode (node);
	nearestNeighbor_ [connectedComponent]->add (node);
	kdTree_.addNode(node);
      }
      return node;
    }

    NodePtr_t Roadmap::addNodeAndEdge (const NodePtr_t from,
				       const ConfigurationPtr_t& to,
				       const PathPtr_t path)
    {
      Path::interval_t timeRange = path->timeRange ();
      NodePtr_t nodeTo = addNode (to, from->connectedComponent ());
      addEdge (from, nodeTo, path);
      addEdge (nodeTo, from, path->extract
	       (Path::interval_t (timeRange.second, timeRange.first)));
      return nodeTo;
    }

    NodePtr_t
    Roadmap::nearestNode (const ConfigurationPtr_t& configuration,
			  const ConnectedComponentPtr_t& connectedComponent,
			  value_type& minDistance)
    {
      NodePtr_t node1;
      NodePtr_t node2;
      if (connectedComponent) {
	node1 = nearestNeighbor_ [connectedComponent]->nearest (configuration, minDistance);
	node2 = kdTree_.search(configuration, connectedComponent, minDistance);
	if ( node1 != node2 ) { throw std::runtime_error ("C'est pas bon, dans ton cul"); }
	else { return node1; }
      } else {
	NodePtr_t closest;
	minDistance = std::numeric_limits<value_type>::infinity ();
	for (ConnectedComponents_t::const_iterator itcc =
	       connectedComponents_.begin ();
	     itcc != connectedComponents_.end (); itcc++) {
	  value_type distance;
	  NodePtr_t node;
	  node1 = nearestNeighbor_ [*itcc]->nearest (configuration, distance);
	  node2 = kdTree_.search(configuration, *itcc, distance);
	  if ( node1 != node2 ) { throw std::runtime_error ("C'est pas bon, dans ton cul"); }
	  else { node = node1; }
	  if (distance < minDistance) {
	    minDistance = distance;
	    closest = node;
	  }
	}
	return closest;
      }
    }

    void Roadmap::addGoalNode (const ConfigurationPtr_t& config)
    {
      NodePtr_t node = addNode (config);
      goalNodes_.push_back (node);
    }

    const DistancePtr_t& Roadmap::distance () const
    {
      return distance_;
    }

    EdgePtr_t Roadmap::addEdge (const NodePtr_t& n1, const NodePtr_t& n2,
				const PathPtr_t& path)
    {
      EdgePtr_t edge = new Edge (n1, n2, path);
      n1->addOutEdge (edge);
      n2->addInEdge (edge);
      edges_.push_back (edge);
      hppDout (info, "Added edge between: " <<
	       displayConfig (*(n1->configuration ())));
      hppDout (info, "               and: " <<
	       displayConfig (*(n2->configuration ())));
      // If node connected components are different, merge them
      ConnectedComponentPtr_t cc1 = n1->connectedComponent ();
      ConnectedComponentPtr_t cc2 = n2->connectedComponent ();
      if (cc1 != cc2) {
	cc1->merge (cc2);
	nearestNeighbor_ [cc1]->merge (nearestNeighbor_ [cc2]);
	// Remove cc2 from list of connected components
	ConnectedComponents_t::iterator itcc =
	  std::find (connectedComponents_.begin (), connectedComponents_.end (),
		     cc2);
	assert (itcc != connectedComponents_.end ());
	connectedComponents_.erase (itcc);
	// remove cc2 from map of nearest neighbors
	NearetNeighborMap_t::iterator itnear = nearestNeighbor_.find (cc2);
	//assert (itnear != nearestNeighbor_.end ());
	nearestNeighbor_.erase (itnear);
      }
      return edge;
    }

    void Roadmap::addConnectedComponent (const NodePtr_t& node)
    {
      connectedComponents_.push_back (node->connectedComponent ());
      nearestNeighbor_ [node->connectedComponent ()] =
	NearestNeighborPtr_t (new NearestNeighbor (distance_));
      node->connectedComponent ()->addNode (node);
      nearestNeighbor_ [node->connectedComponent ()]->add (node);
      kdTree_.addNode(node);
    }

  } //   namespace core
} // namespace hpp
