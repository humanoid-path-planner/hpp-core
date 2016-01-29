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
#include <hpp/model/configuration.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <../src/nearest-neighbor/basic.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    RoadmapPtr_t Roadmap::create (const DistancePtr_t& distance,
				  const DevicePtr_t& robot)
    {
      Roadmap* ptr = new Roadmap (distance, robot);
      return RoadmapPtr_t (ptr);
    }

    Roadmap::Roadmap (const DistancePtr_t& distance, const DevicePtr_t&) :
      distance_ (distance), connectedComponents_ (), nodes_ (), edges_ (),
      initNode_ (), goalNodes_ (),
      nearestNeighbor_ (new nearestNeighbor::Basic (distance))
    {
    }

    Roadmap::~Roadmap ()
    {
      clear ();
    }

    const ConnectedComponents_t& Roadmap::connectedComponents () const
    {
      return connectedComponents_;
    }


    NearestNeighborPtr_t Roadmap::nearestNeighbor(){
      return nearestNeighbor_;
    }


    void Roadmap::nearestNeighbor(NearestNeighborPtr_t nearestNeighbor){
      if (nodes_.size() != 0) {
        throw std::runtime_error ("The roadmap must be empty before setting a new NearestNeighbor object.");
      }
      if(nearestNeighbor)
        nearestNeighbor_ = nearestNeighbor;
    }

    void Roadmap::clear ()
    {
      connectedComponents_.clear ();

      for (Nodes_t::iterator it = nodes_.begin (); it != nodes_.end (); ++it) {
	delete *it;
      }
      nodes_.clear ();

      for (Edges_t::iterator it = edges_.begin (); it != edges_.end (); ++it) {
	delete *it;
      }
      edges_.clear ();

      goalNodes_.clear ();
      initNode_ = 0x0;
      nearestNeighbor_->clear();
    }

    NodePtr_t Roadmap::addNode (const ConfigurationPtr_t& configuration)
    {
      value_type distance;
      if (nodes_.size () != 0) {
	NodePtr_t nearest = nearestNode (configuration, distance);
	if (*(nearest->configuration ()) == *configuration) {
	  return nearest;
	}
      }
      NodePtr_t node = createNode (configuration);
      hppDout (info, "Added node: " << displayConfig (*configuration));
      push_node (node);
      // Node constructor creates a new connected component. This new
      // connected component needs to be added in the roadmap and the
      // new node needs to be registered in the connected component.
      addConnectedComponent (node);
      return node;
    }

    NodePtr_t Roadmap::addNode (const ConfigurationPtr_t& configuration,
				ConnectedComponentPtr_t connectedComponent)
    {
      assert (connectedComponent);
      value_type distance;
      if (nodes_.size () != 0) {
	NodePtr_t nearest = nearestNode (configuration, connectedComponent,
					 distance);
	if (*(nearest->configuration ()) == *configuration) {
	  return nearest;
	}
      }
      NodePtr_t node = createNode (configuration);
      node->connectedComponent (connectedComponent);
      hppDout (info, "Added node: " << displayConfig (*configuration));
      push_node (node);
      // The new node needs to be registered in the connected
      // component.
      connectedComponent->addNode (node);
      nearestNeighbor_->addNode(node);
      return node;
    }

    void Roadmap::addEdges (const NodePtr_t from, const NodePtr_t& to,
			    const PathPtr_t& path)
    {
      EdgePtr_t edge = new Edge (from, to, path);
      if (!from->isOutNeighbor (to)) from->addOutEdge (edge);
      if (!to->isInNeighbor  (from)) to->addInEdge (edge);
      edges_.push_back (edge);
      edge = new Edge (to, from, path->reverse ());
      if (!from->isInNeighbor  (to)) from->addInEdge (edge);
      if (!to->isOutNeighbor (from)) to->addOutEdge (edge);
      edges_.push_back (edge);
    }

    NodePtr_t Roadmap::addNodeAndEdges (const NodePtr_t from,
					const ConfigurationPtr_t& to,
					const PathPtr_t path)
    {
      NodePtr_t nodeTo = addNode (to, from->connectedComponent ());
      addEdges (from, nodeTo, path);
      return nodeTo;
    }

    NodePtr_t
    Roadmap::nearestNode (const ConfigurationPtr_t& configuration,
			  value_type& minDistance)
    {
      NodePtr_t closest = 0x0;
      minDistance = std::numeric_limits<value_type>::infinity ();
      for (ConnectedComponents_t::const_iterator itcc =
	     connectedComponents_.begin ();
	   itcc != connectedComponents_.end (); ++itcc) {
	value_type distance;
	NodePtr_t node;
	node = nearestNeighbor_->search(configuration, *itcc, distance);
	if (distance < minDistance) {
	  minDistance = distance;
	  closest = node;
	}
      }
      return closest;
    }

    NodePtr_t
    Roadmap::nearestNode (const ConfigurationPtr_t& configuration,
			  const ConnectedComponentPtr_t& connectedComponent,
			  value_type& minDistance)
    {
      assert (connectedComponent);
      assert (connectedComponent->nodes ().size () != 0);
      NodePtr_t closest =
	nearestNeighbor_->search(configuration, connectedComponent, minDistance);
      return closest;
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
      if (!n1->isOutNeighbor (n2)) n1->addOutEdge (edge);
      if (!n2->isInNeighbor  (n1)) n2->addInEdge (edge);
      edges_.push_back (edge);

      ConnectedComponentPtr_t cc1 = n1->connectedComponent ();
      ConnectedComponentPtr_t cc2 = n2->connectedComponent ();

      connect (cc1, cc2);
      return edge;
    }

    void Roadmap::addConnectedComponent (const NodePtr_t& node)
    {
      connectedComponents_.insert (node->connectedComponent ());
      node->connectedComponent ()->addNode (node);
      nearestNeighbor_->addNode(node);
    }

    NodePtr_t Roadmap::createNode (const ConfigurationPtr_t& configuration)
      const
    {
      return NodePtr_t (new Node (configuration));
    }

    void Roadmap::connect (const ConnectedComponentPtr_t& cc1,
			   const ConnectedComponentPtr_t& cc2)
    {
      if (cc1->canReach (cc2)) return;
      ConnectedComponents_t cc2Tocc1;
      if (cc2->canReach (cc1, cc2Tocc1)) {
	merge (cc1, cc2Tocc1);
      } else {
	cc1->reachableTo_.insert (cc2);
	cc2->reachableFrom_.insert (cc1);
      }
    }
  
    void Roadmap::merge (const ConnectedComponentPtr_t& cc1,
			 ConnectedComponents_t& ccs)
    {
      for (ConnectedComponents_t::iterator itcc = ccs.begin ();
	   itcc != ccs.end (); ++itcc) {
	if (*itcc != cc1) {
	  cc1->merge (*itcc);
#ifndef NDEBUG	  
	  std::size_t nb =
#endif
	    connectedComponents_.erase (*itcc);
	  assert (nb == 1);
	}
      }
    }

    bool Roadmap::pathExists () const
    {
      const ConnectedComponentPtr_t ccInit = initNode ()->connectedComponent ();
      for (Nodes_t::const_iterator itGoal = goalNodes_.begin ();
	   itGoal != goalNodes_.end (); ++itGoal) {
	if (ccInit->canReach ((*itGoal)->connectedComponent ())) {
	  return true;
	}
      }
      return false;
    }

    std::ostream& Roadmap::print (std::ostream& os) const
    {
      // Enumerate nodes and connected components
      std::map <NodePtr_t, size_type> nodeId;
      std::map <ConnectedComponentPtr_t, size_type> ccId;
      std::map <ConnectedComponentPtr_t, size_type> sccId;

      size_type count = 0;
      for (Nodes_t::const_iterator it = nodes ().begin ();
	   it != nodes ().end (); ++it) {
	nodeId [*it] = count; ++count;
      }

      count = 0;
      for (ConnectedComponents_t::const_iterator it =
	     connectedComponents ().begin ();
	   it != connectedComponents ().end (); ++it) {
	ccId [*it] = count; ++count;
      }


      // Display list of nodes
      os << "-----------------------------------------------------------------"
	 << std::endl;
      os << "Roadmap" << std::endl;
      os << "-----------------------------------------------------------------"
	 << std::endl;
      os << "-----------------------------------------------------------------"
	 << std::endl;
      os << "Nodes" << std::endl;
      os << "-----------------------------------------------------------------"
	 << std::endl;
      for (Nodes_t::const_iterator it = nodes ().begin ();
	   it != nodes ().end (); ++it) {
	const NodePtr_t node = *it;
	os << "Node " << nodeId [node] << ": " << *node << std::endl;
      }
      os << "-----------------------------------------------------------------"
	 << std::endl;
      os << "Edges" << std::endl;
      os << "-----------------------------------------------------------------"
	 << std::endl;
      for (Edges_t::const_iterator it = edges ().begin ();
	   it != edges ().end (); ++it) {
	const EdgePtr_t edge = *it;
	os << "Edge: " << nodeId [edge->from ()] << " -> "
	   << nodeId [edge->to ()] << std::endl;
      }
      os << "-----------------------------------------------------------------"
	 << std::endl;
      os << "Connected components" << std::endl;
      os << "-----------------------------------------------------------------"
	 << std::endl;
      for (ConnectedComponents_t::const_iterator it =
	     connectedComponents ().begin ();
	   it != connectedComponents ().end (); ++it) {
	const ConnectedComponentPtr_t cc = *it;
	os << "Connected component " << ccId [cc] << std::endl;
	os << "Nodes : ";
	for (NodeVector_t::const_iterator itNode = cc->nodes ().begin ();
	     itNode != cc->nodes ().end (); ++itNode) {
	  os << nodeId [*itNode] << ", ";
	}
	os << std::endl;
	os << "Reachable to :";
	for (ConnectedComponents_t::const_iterator itTo =
	       cc->reachableTo ().begin (); itTo != cc->reachableTo ().end ();
	     ++itTo) {
	  os << ccId [*itTo] << ", ";
	}
	os << std::endl;
	os << "Reachable from :";
	for (ConnectedComponents_t::const_iterator itFrom =
	       cc->reachableFrom ().begin ();
	     itFrom != cc->reachableFrom ().end (); ++itFrom) {
	  os << ccId [*itFrom] << ", ";
	}
	os << std::endl;
      }
      os << std::endl;
      os << "----------------" << std::endl;

      return os;
    }

    std::ostream& operator<< (std::ostream& os, const hpp::core::Roadmap& r)
    {
      return r.print (os);
    }
  } //   namespace core
} // namespace hpp

