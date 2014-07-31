//
// Copyright (c) 2014 CNRS
// Authors: Mukunda Bharatheesha
//          Florent Lamiraux
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
#include <hpp/core/graph-connected-component.hh>

namespace hpp {
  namespace core {

    void CCGraph::updateCCReachability (const ConnectedComponentPtr_t& cc1,
					const ConnectedComponentPtr_t& cc2)
    {
      //Update the respective reachability lists of the connected components
      //CC Iterator for connectivity update
      ConnectedComponents_t::iterator itcc;

      itcc = cc1->reachableTo_.end ();
      itcc = cc1->reachableTo_.insert (itcc, cc2);
      //Remove multiple copies
      cc1->reachableTo_.sort ();cc1->reachableTo_.unique ();

      itcc = cc2->reachableFrom_.end ();
      itcc = cc2->reachableFrom_.insert (itcc, cc1);
      //Remove multiple copies
      cc2->reachableFrom_.sort ();cc2->reachableFrom_.unique ();
    }

    /// Finds Strongly Connected Components (SCCs) in the CCGraph
    /// using "Kosaraju's Two-Pass algorithm".
    void CCGraph::findSCC ()
    {
      ConnectedComponents_t::iterator itcc;
      ConnectedComponents_t::iterator itSCCHeads;

      itSCCHeads = sccHeadsList_.begin ();
      //variable to indicate direction of DFS
      bool reverseDFS_ = true;
      //Search for loops in a reverse manner using DFS
      for (itcc = connectedComponents_.begin ();
	   itcc != connectedComponents_.end ();
	   itcc++) {
	if (!((*itcc)->isExplored ())) {
	  setSCCHead (*itcc);
	  DFS (*itcc, reverseDFS_);
	}
      }
      //Sort the connected components in decreasing order
      //of finish time
      connectedComponents_.sort
	(ConnectedComponent::compareCCFinishTime ());

      //Clear the leader list for the forward DFS
      sccHeadsList_.clear ();

      //Reset explored status of all CCs
      for (itcc = connectedComponents_.begin ();
	   itcc != connectedComponents_.end ();
	   itcc++) {
	(*itcc)->resetExplored ();
      }

      itSCCHeads = sccHeadsList_.begin ();
      bool isAdvanced_;
      //Reset the reverse DFS flag
      reverseDFS_ = false;

      //Search for loops in a forward manner using DFS
      for (itcc = connectedComponents_.begin ();
	   itcc != connectedComponents_.end ();
	   itcc++) {
	if (!((*itcc)->isExplored ())) {
	  setSCCHead (*itcc);
	  (*itcc)->setLeader(*itcc);
	  isAdvanced_=false;
	  DFS (*itcc, reverseDFS_);
	}
	else {
	  if(!isAdvanced_) {
	    std::advance(itSCCHeads, 1);
	    isAdvanced_ = true;
	  }
	  //Merge mutually reachable CCs
	  (*itcc)->setLeader (*(itSCCHeads));
	  if ( itSCCHeads != sccHeadsList_.end () ) {
	    (*itSCCHeads)->merge (*itcc);
	  }
	}
      }
      //Remove the merged connected components
      for (itcc = sccHeadsList_.begin ();
	   itcc != sccHeadsList_.end ();
	   itcc++) {
	for (ConnectedComponents_t::iterator itcc1 =
	       (*itcc)->reachableTo_.begin ();
	     itcc1 != (*itcc)->reachableTo_.end ();
	     itcc1++) {
	  ConnectedComponents_t::iterator itLeader;
	  itLeader = std::find (sccHeadsList_.begin (),
				sccHeadsList_.end (),
				(*itcc1)->getLeader ());
	  *itcc1 = *itLeader;
	}
	(*itcc)->reachableTo_.sort ();
	(*itcc)->reachableTo_.unique();
	for (ConnectedComponents_t::iterator itcc1 =
	       (*itcc)->reachableFrom_.begin ();
	     itcc1 != (*itcc)->reachableFrom_.end ();
	     itcc1++) {
	  ConnectedComponents_t::iterator itLeader;
	  itLeader = std::find (sccHeadsList_.begin (),
				sccHeadsList_.end (),
				(*itcc1)->getLeader ());
	  *itcc1 = *itLeader;
	}
	(*itcc)->reachableFrom_.sort ();
	(*itcc)->reachableFrom_.unique();
      }
      connectedComponents_.remove_if
	(ConnectedComponent::emptyCC ());

      //Clear the leader list for the forward DFS
      sccHeadsList_.clear ();

      //Reset explored status of all CCs
      for (itcc = connectedComponents_.begin ();
	   itcc != connectedComponents_.end ();
	   itcc++) {
	(*itcc)->resetExplored ();
      }
      //Reset current ranks of all CCs
      for (itcc = connectedComponents_.begin ();
	   itcc != connectedComponents_.end ();
	   itcc ++) {
	(*itcc)->resetFinishTime ();
      }
      ConnectedComponent::globalFinishTime_ = 0;
    }

    unsigned int ConnectedComponent::globalFinishTime_ = 0;

    void CCGraph::setSCCHead
    (const ConnectedComponentPtr_t& headCC)
    {
      sccHeadsList_.push_back (headCC);
    }

    void CCGraph::DFS
    (const ConnectedComponentPtr_t& cc, bool reverse)
    {
      ConnectedComponents_t::iterator itcc_dfs;
      cc->setExplored ();

      if (reverse) {
	//The reverse CC Graph traversal is accomplished
	//by using the reachableFrom list of the connected
	//components
	for (itcc_dfs = cc->reachableFrom_.begin (); itcc_dfs != cc->reachableFrom_.end ();
	     itcc_dfs++) {
	  if (!((*itcc_dfs)->isExplored ())) {
	    DFS (*itcc_dfs, reverse);
	  }
	}
      }
      else {
	for (itcc_dfs = cc->reachableTo_.begin (); itcc_dfs != cc->reachableTo_.end ();
	     itcc_dfs++) {
	  if (!((*itcc_dfs)->isExplored ())) {
	    DFS (*itcc_dfs, reverse);
	  }
	}
      }
      ConnectedComponent::globalFinishTime_++;
      cc->setFinishTime();
    }

  } //    namespace core
}// namespace hpp
