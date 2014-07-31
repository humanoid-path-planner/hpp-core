//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//          Mukunda Bharatheesha
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

#ifndef HPP_CORE_GRAPH_CONNECTED_COMPONENT_HH
# define HPP_CORE_GRAPH_CONNECTED_COMPONENT_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/connected-component.hh>

namespace hpp {
  namespace core {
    /// Graph of Connected components
    ///
    /// List of all (strongly) connected components
    /// List of connected components that can be reached from
    /// a given connected component
    /// List of connected components that reach a given connected
    /// component
    class HPP_CORE_DLLAPI CCGraph {
    public:
      static CCGraphPtr_t create ()
      {
	CCGraph* ptr = new CCGraph ();
	CCGraphPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      void addConnectedComponent (const ConnectedComponentPtr_t& cc)
      {
	connectedComponents_.push_back (cc);
      }
      /// Check and update reachability between two connected components
      /// \param connectedComponent1: the first connected component
      /// \param connectedComponent2: the second connected component
      /// If both connected components are reachable to each other,
      /// they are merged. Else, their respective reachability lists
      /// are updated
      void updateCCReachability (const ConnectedComponentPtr_t&
				 connectedComponent1, const ConnectedComponentPtr_t&
				 connectedComponent2);


      /// Find Strongly Connected Components (SCC) in roadmap
      /// \param roadMap: entire roadmap for finding SCC
      void findSCC ();

      void clear ()
      {
	connectedComponents_.clear ();
	sccHeadsList_.clear ();
      }

      void setSCCHead (const ConnectedComponentPtr_t& headCC);

      void DFS (const ConnectedComponentPtr_t& cc, bool reverse);

      ///Access to connected components
      const ConnectedComponents_t& connectedComponents () const
      {
	return connectedComponents_;
      }
      const ConnectedComponents_t& sccHeads () const
      {
	return sccHeadsList_;
      }

    protected:
      /// Constructor
      CCGraph () : connectedComponents_ (), sccHeadsList_ (),
	weak_ ()
	  {
	  }
      void init (const CCGraphPtr_t& shPtr){
	weak_ = shPtr;
      }
    private:
      //The complete list of (strongly) connected components
      ConnectedComponents_t connectedComponents_;

      /// Elements needed for SCC detection algorithm
      // Leader connected component
      ConnectedComponents_t sccHeadsList_;
      CCGraphWkPtr_t weak_;
    }; // class CCGraph
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_GRAPH_CONNECTED_COMPONENT_HH
