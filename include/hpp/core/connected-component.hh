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

#ifndef HPP_CORE_CONNECTED_COMPONENT_HH
# define HPP_CORE_CONNECTED_COMPONENT_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/node.hh>

namespace hpp {
  namespace core {
    /// Connected component
    ///
    /// Set of nodes reachable from one another.
    class HPP_CORE_DLLAPI ConnectedComponent {
    public:
      // variable for ranking connected components
      static unsigned int globalFinishTime_;
      static ConnectedComponentPtr_t create ()
      {
	ConnectedComponent* ptr = new ConnectedComponent ();
	ConnectedComponentPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Merge two connected components.
      ///
      /// \param other connected component to merge into this one.
      /// \note other will be empty after calling this method.
      virtual void merge (const ConnectedComponentPtr_t& other);
      
      virtual ~ConnectedComponent() {}
      
      /// Add node in connected component
      /// \param node node to add.
      virtual void addNode (const NodePtr_t& node)
      {
	nodes_.push_back (node);
      }
      /// Access to the nodes
      const NodeVector_t& nodes () const
      {
	return nodes_;
      }

      /// \name Reachability
      /// \{

      /// Whether this connected component can reach cc
      /// \param cc a connected component
      bool canReach (const ConnectedComponentPtr_t& cc);

      
      /// Whether this connected component can reach cc
      /// \param cc a connected component
      /// \retval cc2Tocc1 list of connected components between cc2 and cc1
      ///         that should be merged.
      bool canReach (const ConnectedComponentPtr_t& cc,
		     ConnectedComponents_t& cc2Tocc1);

      // Get connected components reachable from this
      const ConnectedComponents_t& reachableTo () const
      {
	return reachableTo_;
      }

      // Get connected components that can reach this
      const ConnectedComponents_t& reachableFrom () const
      {
	return reachableFrom_;
      }
      /// \}
    protected:
      /// Constructor
      ConnectedComponent () : nodes_ (), explored_ (false), weak_ ()
	  {
            nodes_.reserve (1000);
	  }
      void init (const ConnectedComponentPtr_t& shPtr){
	weak_ = shPtr;
      }
    private:
      NodeVector_t nodes_;
      // List of CCs from which this connected component can be reached
      ConnectedComponents_t reachableFrom_;
      // List of CCs that can be reached from this connected component
      ConnectedComponents_t reachableTo_;
      // status variable to indicate whether or not CC has been visited
      mutable bool explored_;
      ConnectedComponentWkPtr_t weak_;
      friend class Roadmap;
      friend void clean (ConnectedComponents_t& set);
    }; // class ConnectedComponent
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_CONNECTED_COMPONENT_HH
