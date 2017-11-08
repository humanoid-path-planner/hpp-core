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
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>

namespace hpp {
  namespace core {
    // Mark connected components of a list as unexplored.
    void ConnectedComponent::clean (RawPtrs_t& set)
    {
      for (RawPtrs_t::iterator it = set.begin ();
	   it != set.end (); ++it) {
	(*it)->explored_ = false;
      }
    }

    void ConnectedComponent::merge (const ConnectedComponentPtr_t& other)
    {
      assert(other);
      assert(weak_.lock().get() == this);

      // Tell other's nodes that they now belong to this connected component
      for (NodeVector_t::iterator itNode = other->nodes_.begin ();
	   itNode != other->nodes_.end (); ++itNode) {
	(*itNode)->connectedComponent (weak_.lock ());
      }
      // Add other's nodes to this list.
      nodes_.insert (nodes_.end (), other->nodes_.begin(), other->nodes_.end());

      // Tell other's reachableTo's that other has been replaced by this
      for (RawPtrs_t::iterator itcc = other->reachableTo_.begin ();
	   itcc != other->reachableTo_.end (); ++itcc) {
	(*itcc)->reachableFrom_.erase (other.get());
	(*itcc)->reachableFrom_.insert (this);
      }

      // Tell other's reachableFrom's that other has been replaced by this
      for (RawPtrs_t::iterator itcc=other->reachableFrom_.begin ();
	   itcc != other->reachableFrom_.end (); ++itcc) {
	(*itcc)->reachableTo_.erase (other.get());
	(*itcc)->reachableTo_.insert (this);
      }
      
      RawPtrs_t tmp;
      std::set_union (reachableTo_.begin (), reachableTo_.end (),
		      other->reachableTo_.begin (), other->reachableTo_.end (),
		      std::inserter (tmp, tmp.begin ()));
      reachableTo_ = tmp; tmp.clear ();
      reachableTo_.erase (other.get());
      reachableTo_.erase (this);
      std::set_union (reachableFrom_.begin (), reachableFrom_.end (),
		      other->reachableFrom_.begin (),
		      other->reachableFrom_.end (),
		      std::inserter (tmp, tmp.begin()));
      reachableFrom_ = tmp; tmp.clear ();
      reachableFrom_.erase (other.get());
      reachableFrom_.erase (this);
    }

    bool ConnectedComponent::canReach (const ConnectedComponentPtr_t& cc)
    {
      // Store visited connected components for further cleaning.
      RawPtrs_t explored;
      std::deque <RawPtr_t> queue;
      queue.push_back (this);
      explored_ = true;
      explored.insert (this);
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == cc.get()) {
	  clean (explored);
	  return true;
	}
	for (RawPtrs_t::iterator itChild =
	       current->reachableTo_.begin ();
	     itChild != current->reachableTo_.end (); ++itChild) {
	  RawPtr_t child = *itChild;
	  if (!child->explored_) {
	    child->explored_ = true;
	    explored.insert (child);
	    queue.push_back (child);
	  }
	}
      }
      clean (explored);
      return false;
    }

    bool ConnectedComponent::canReach
    (const ConnectedComponentPtr_t& cc, RawPtrs_t& ccToThis)
    {
      bool reachable = false;
      // Store visited connected components
      RawPtrs_t exploredForward;
      std::deque <RawPtr_t> queue;
      queue.push_back (this);
      explored_ = true;
      exploredForward.insert (this);
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == cc.get()) {
	  reachable = true;
	  exploredForward.insert (current);
	} else {
	  for (RawPtrs_t::iterator itChild =
		 current->reachableTo_.begin ();
	       itChild != current->reachableTo_.end (); ++itChild) {
	    RawPtr_t child = *itChild;
	    if (!child->explored_) {
	      child->explored_ = true;
	      exploredForward.insert (child);
	      queue.push_back (child);
	    }
	  }
	}
      }
      // Set visited connected components to unexplored
      clean (exploredForward);
      if (!reachable) return false;

      // Store visited connected components
      RawPtrs_t exploredBackward;
      queue.push_back (cc.get());
      cc->explored_ = true;
      exploredBackward.insert (cc.get());
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == this) {
	  exploredBackward.insert (current);
	} else {
	  for (RawPtrs_t::iterator itChild =
		 current->reachableFrom_.begin ();
	       itChild != current->reachableFrom_.end (); ++itChild) {
	    RawPtr_t child = *itChild;
	    if (!child->explored_) {
	      child->explored_ = true;
	      exploredBackward.insert (child);
	      queue.push_back (child);
	    }
	  }
	}
      }
      // Set visited connected components to unexplored
      clean (exploredBackward);
      std::set_intersection (exploredForward.begin (), exploredForward.end (),
			     exploredBackward.begin (), exploredBackward.end (),
			     std::inserter (ccToThis, ccToThis.begin ()));
      return true;
    }

  } //   namespace core
} // namespace hpp
