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

#include "../src/nearest-neighbor/basic.hh"

# include <limits>
# include <queue>

# include <hpp/core/distance.hh>
# include <hpp/core/connected-component.hh>

namespace hpp {
  namespace core {
    namespace nearestNeighbor {
      namespace {
        typedef std::pair <value_type, NodePtr_t> DistAndNode_t;
        struct DistAndNodeComp_t {
          bool operator () (const DistAndNode_t& r,
              const DistAndNode_t& l) {
            return r.first < l.first;
          }
        };
        typedef std::priority_queue <DistAndNode_t, std::vector <DistAndNode_t>,
                DistAndNodeComp_t > Queue_t;
      }

      NodePtr_t Basic::search (const ConfigurationPtr_t& configuration,
             const ConnectedComponentPtr_t&
        connectedComponent,
             value_type& distance, bool reverse)
      {
	NodePtr_t result = NULL;
	distance = std::numeric_limits <value_type>::infinity ();
        const Distance& dist = *distance_;
        value_type d;
	for (NodeVector_t::const_iterator itNode =
	       connectedComponent->nodes ().begin ();
	     itNode != connectedComponent->nodes ().end (); ++itNode) {
    if(reverse)
      d = dist ( *configuration, *(*itNode)->configuration ());
    else
      d = dist ( *(*itNode)->configuration (), *configuration);
	  if (d < distance) {
	    distance = d;
	    result = *itNode;
	  }
	}
	assert (result);
	return result;
      }

      NodePtr_t Basic::search (const NodePtr_t& node,
             const ConnectedComponentPtr_t&
        connectedComponent,
             value_type& distance)
      {
	NodePtr_t result = NULL;
	distance = std::numeric_limits <value_type>::infinity ();
        const Distance& dist = *distance_;
	for (NodeVector_t::const_iterator itNode =
	       connectedComponent->nodes ().begin ();
	     itNode != connectedComponent->nodes ().end (); ++itNode) {
	  value_type d = dist (*itNode, node);
	  if (d < distance) {
	    distance = d;
	    result = *itNode;
	  }
	}
	assert (result);
	return result;
      }

      Nodes_t Basic::KnearestSearch (const ConfigurationPtr_t& configuration,
          const ConnectedComponentPtr_t&
          connectedComponent,
          const std::size_t K,
          value_type& distance)
      {
        Queue_t ns;
        distance = std::numeric_limits <value_type>::infinity ();
        const Distance& dist = *distance_;
        const Configuration_t& q = *configuration;
        for (NodeVector_t::const_iterator itNode =
            connectedComponent->nodes ().begin ();
            itNode != connectedComponent->nodes ().end (); ++itNode) {
          value_type d = dist (*(*itNode)->configuration (), q);
          if (ns.size () < K)
            ns.push (DistAndNode_t (d, (*itNode)));
          else if (ns.top().first > d) {
            ns.pop ();
            ns.push (DistAndNode_t (d, (*itNode)));
          }
        }
        Nodes_t nodes;
        if (ns.size() > 0) distance = ns.top ().first;
        while (ns.size () > 0) {
          nodes.push_front (ns.top().second); ns.pop ();
        }
        return nodes;
      }

      Nodes_t Basic::KnearestSearch (const NodePtr_t& node,
          const ConnectedComponentPtr_t&
          connectedComponent,
          const std::size_t K,
          value_type& distance)
      {
        Queue_t ns;
        distance = std::numeric_limits <value_type>::infinity ();
        const Distance& dist = *distance_;
        for (NodeVector_t::const_iterator itNode =
            connectedComponent->nodes ().begin ();
            itNode != connectedComponent->nodes ().end (); ++itNode) {
          value_type d = dist (*itNode, node);
          if (ns.size () < K)
            ns.push (DistAndNode_t (d, (*itNode)));
          else if (ns.top().first > d) {
            ns.pop ();
            ns.push (DistAndNode_t (d, (*itNode)));
          }
        }
        Nodes_t nodes;
        if (ns.size() > 0) distance = ns.top ().first;
        while (ns.size () > 0) {
          nodes.push_front (ns.top().second); ns.pop ();
        }
        return nodes;
      }
    } // namespace nearestNeighbor
  } // namespace core
} // namespace hpp
