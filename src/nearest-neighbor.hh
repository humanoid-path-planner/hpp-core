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

#ifndef HPP_CORE_NEAREST_NEIGHBOR_HH
# define HPP_CORE_NEAREST_NEIGHBOR_HH

# include <limits>
# include <hpp/core/fwd.hh>
# include <hpp/core/distance.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (NearestNeighbor);
    typedef boost::shared_ptr <NearestNeighbor> NearestNeighborPtr_t;

    /// Optimization of the nearest neighbor search
    class NearestNeighbor
    {
    public:

      NearestNeighbor(const DistancePtr_t& distance) : distance_ (distance)
      {
      }

      ~NearestNeighbor()
      {
      }

      // void distance (const DistancePtr_t& distance)
      // {
      // 	distance_ = distance;
      // }

      DistancePtr_t distance () const
      {
	return distance_;
      }

      void clear ()
      {
	nodes_.clear ();
      }

      void add (const NodePtr_t& node)
      {
	nodes_.push_back (node);
      }

      void merge (const NearestNeighborPtr_t& other)
      {
	nodes_.splice (nodes_.end (), other->nodes_);
      }

      bool remove (const NodePtr_t& node)
      {
	Nodes_t::iterator itConfig =
	  std::find (nodes_.begin (), nodes_.end (), node);
	if (itConfig == nodes_.end ()) {
	  throw std::runtime_error ("Attempt to remove a node that is"
				    " not in nearest neighbor data-structure.");
	}
	nodes_.erase (itConfig);
      }

      NodePtr_t nearest (const ConfigurationPtr_t& configuration,
			 value_type& minDistance)
      {
	assert (!nodes_.empty ());
	minDistance = std::numeric_limits <value_type>::infinity ();
	Nodes_t::iterator itClosest = nodes_.end ();
	for (Nodes_t::iterator itNode = nodes_.begin ();
	     itNode != nodes_.end (); itNode ++) {
	  value_type distance = (*distance_) (*configuration,
					  *((*itNode)->configuration ()));
	  if (distance < minDistance) {
	    minDistance = distance;
	    itClosest = itNode;
	  }
	}
	assert (itClosest != nodes_.end ());
	return *itClosest;
      }

      ///  Return the k nearest neighbors in sorted order
      void nearest (const ConfigurationPtr_t& configuration, std::size_t k,
		    Nodes_t& nearestNeighbors) const
      {
	Nodes_t copy;
	std::copy (nodes_.begin (), nodes_.end (), copy.begin ());
	nearestNeighbors.clear ();
	while (nearestNeighbors.size () < k) {
	  value_type minDistance = std::numeric_limits <value_type>::infinity ();
	  Nodes_t::iterator itClosest = copy.end ();
	  for (Nodes_t::iterator itNode = copy.begin ();
	       itNode != copy.end (); itNode ++) {
	    value_type distance = (*distance_) (*configuration,
					    *((*itNode)->configuration ()));
	    if (distance < minDistance) {
	      minDistance = distance;
	      itClosest = itNode;
	    }
	  }
	  if (itClosest != copy.end ()) {
	    nearestNeighbors.push_back (*itClosest);
	    copy.erase (itClosest);
	  } else {
	    k = 0; // if k is more than number of configs, stop
	  }
	}
      }

    private:
      Nodes_t nodes_;
      const DistancePtr_t& distance_;
    }; // class NearestNeighbor
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_NEAREST_NEIGHBOR_HH
