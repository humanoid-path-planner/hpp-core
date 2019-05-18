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

#ifndef HPP_CORE_ROADMAP_HH
# define HPP_CORE_ROADMAP_HH

# include <iostream>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

# include <hpp/core/connected-component.hh>

namespace hpp {
  namespace core {
    /// \addtogroup roadmap
    /// \{

    /// Roadmap built by random path planning methods
    /// Nodes are configurations, paths are collision-free paths.
    class HPP_CORE_DLLAPI Roadmap {
    public:
      /// Return shared pointer to new instance.
      static RoadmapPtr_t create (const DistancePtr_t& distance, const DevicePtr_t& robot);

      /// Clear the roadmap by deleting nodes and edges.
      virtual void clear ();

      /// Add a node with given configuration
      /// \param config configuration
      ///
      /// If configuration is alread in the roadmap, return the node
      /// containing the configuration. Otherwise, create a new node and a new
      /// connected component with this node.
      NodePtr_t addNode (const ConfigurationPtr_t& config);

      NodePtr_t addNode (const Configuration_t& config)
      {
        return addNode (ConfigurationPtr_t (new Configuration_t(config)));
      }

      /// Get nearest node to a configuration in the roadmap.
      /// \param configuration configuration
      /// \param reverse if true, compute distance from given configuration to nodes in roadmap,
      /// if false from nodes in roadmap to given configuration
      /// \retval distance to the nearest node.
      NodePtr_t nearestNode (const Configuration_t& configuration,
           value_type& minDistance, bool reverse = false);

      NodePtr_t nearestNode (const ConfigurationPtr_t& configuration,
           value_type& minDistance, bool reverse = false)
      {
        return nearestNode (*configuration, minDistance, reverse);
      }

      /// Get nearest node to a configuration in a connected component.
      /// \param configuration configuration
      /// \param connectedComponent the connected component
      /// \param reverse if true, compute distance from given configuration to nodes in roadmap,
      /// if false from nodes in roadmap to given configuration
      /// \retval distance to the nearest node.
      NodePtr_t nearestNode (const Configuration_t& configuration,
			     const ConnectedComponentPtr_t& connectedComponent,
           value_type& minDistance, bool reverse = false);

      NodePtr_t nearestNode (const ConfigurationPtr_t& configuration,
			     const ConnectedComponentPtr_t& connectedComponent,
           value_type& minDistance, bool reverse = false)
      {
        return nearestNode (*configuration, connectedComponent, minDistance, reverse);
      }

      /// Get nearest node to a configuration in the roadmap.
      /// \param configuration configuration
      /// \param k number of nearest nodes to return
      /// if false from nodes in roadmap to given configuration
      /// \return k nearest nodes
      Nodes_t nearestNodes (const Configuration_t& configuration,
                            size_type k);

      Nodes_t nearestNodes (const ConfigurationPtr_t& configuration,
                            size_type k)
      {
        return nearestNodes (*configuration, k);
      }

      /// Get nearest node to a configuration in a connected component.
      /// \param configuration configuration
      /// \param connectedComponent the connected component
      /// \param k number of nearest nodes to return
      /// if false from nodes in roadmap to given configuration
      /// \return k nearest nodes in the connected component
      Nodes_t nearestNodes (const Configuration_t& configuration,
                            const ConnectedComponentPtr_t&
                            connectedComponent,
                            size_type k);

      Nodes_t nearestNodes (const ConfigurationPtr_t& configuration,
                            const ConnectedComponentPtr_t&
                            connectedComponent,
                            size_type k)
      {
        return nearestNodes (*configuration, connectedComponent, k);
      }

      /// Add a node and two edges
      /// \param from node from which the edge starts,
      /// \param to configuration to which the edge stops
      /// \param path path between both configurations
      /// \return node containing configuration <c>to</c>.
      /// Add the symmetric edge with reverse path.
      /// \note this function simplifies the management of connected components
      ///       since it adds the new node in the connected component of
      ///       <c>from</c>.
      NodePtr_t addNodeAndEdges (const NodePtr_t from,
				 const ConfigurationPtr_t& to,
				 const PathPtr_t path);

      /// Add a node and one edge
      /// \param from node from which the edge starts,
      /// \param to configuration to which the edge stops
      /// \param path path between both configurations
      /// \return node containing configuration <c>to</c>.
      /// Add the oriented edge (from -> to)
      /// \note this function simplifies the management of connected components
      ///       since it adds the new node in the connected component of
      ///       <c>from</c>.
      NodePtr_t addNodeAndEdge (const NodePtr_t from,
                 const ConfigurationPtr_t& to,
                 const PathPtr_t path);

      /// Add a node and one edge
      /// \param from configuration from which the edge starts,
      /// \param to node to which the edge stops
      /// \param path path between both configurations
      /// \return node containing configuration <c>from</c>.
      /// Add the oriented edge (from -> to)
      /// \note this function simplifies the management of connected components
      ///       since it adds the new node in the connected component of
      ///       <c>to</c>.
      NodePtr_t addNodeAndEdge (const ConfigurationPtr_t& from,
                      const NodePtr_t to,
                      const PathPtr_t path);


      /// Add an edge between two nodes.
      EdgePtr_t addEdge (const NodePtr_t& n1, const NodePtr_t& n2,
			 const PathPtr_t& path);

      /// Add two edges between two nodes
      /// \param from first node
      /// \param to second node
      /// \param path path going from <c>from</c> to <c>to</c>.
      /// the reverse edge is added with the reverse path.
      void addEdges (const NodePtr_t from, const NodePtr_t& to,
		     const PathPtr_t& path);

      /// Add a goal configuration
      /// \param config configuration
      /// If configuration is already in the roadmap, tag corresponding node
      /// as goal node. Otherwise create a new node.
      NodePtr_t addGoalNode (const ConfigurationPtr_t& config);

      void resetGoalNodes ()
      {
	goalNodes_.clear ();
      }

      void initNode (const ConfigurationPtr_t& config)
      {
	initNode_ = addNode (config);
      }

      virtual ~Roadmap ();
      /// Check that a path exists between the initial node and one goal node.
      bool pathExists () const;
      const Nodes_t& nodes () const
      {
	return nodes_;
      }
      const Edges_t& edges () const
      {
	return edges_;
      }
      NodePtr_t initNode () const
      {
	return initNode_;
      }
      const NodeVector_t& goalNodes () const
      {
	return goalNodes_;
      }
      /// Get list of connected component of the roadmap
      const ConnectedComponents_t& connectedComponents () const;

      /// Get nearestNeighbor object
      NearestNeighborPtr_t nearestNeighbor();

      /// Set new NearestNeighbor (roadmap must be empty)
      void nearestNeighbor(NearestNeighborPtr_t nearestNeighbor);

      /// \name Distance used for nearest neighbor search
      /// \{
      /// Get distance function
      const DistancePtr_t& distance () const;
      /// \}
      /// Print roadmap in a stream
      std::ostream& print (std::ostream& os) const;

    protected:
      /// Constructor
      /// \param distance distance function for nearest neighbor computations
      Roadmap (const DistancePtr_t& distance, const DevicePtr_t& robot);

      /// Add a new connected component in the roadmap.
      /// \param node node pointing to the connected component.
      /// \note The node is added in the connected component.
      void addConnectedComponent (const NodePtr_t& node);

      /// Give child class the opportunity to get the event
      /// "A node has been added to the roadmap"
      virtual void push_node (const NodePtr_t& n)
      {
        nodes_.push_back (n);
      }

      /// Give child class the opportunity to get the event
      /// "An edge has been added to the roadmap"
      /// \note you must always call the parent implementation first
      /// \code
      /// void YourRoadmap::push_edge(const EdgePtr_t e) {
      ///   Roadmap::push_edge(e);
      ///   // Your code here.
      /// }
      /// \endcode
      virtual void addEdge (const EdgePtr_t& e);

      /// Node factory
      /// Reimplement the function if you want to create an instance of a
      /// child class of Node
      virtual NodePtr_t createNode (const ConfigurationPtr_t& configuration) const;

      /// Store weak pointer to itself
      void init (RoadmapWkPtr_t weak);

    private:
      /// Add a node with given configuration
      /// \param config configuration
      /// \param connectedComponent Connected component the node will belong
      ///        to.
      ///
      /// If configuration is alread in the connected component, return the node
      /// containing the configuration. Otherwise, create a new node with given
      /// connected component.
      NodePtr_t addNode (const ConfigurationPtr_t& config,
			 ConnectedComponentPtr_t connectedComponent);

      /// Update the graph of connected components after new connection
      /// \param cc1, cc2 the two connected components that have just been
      /// connected.
      void connect (const ConnectedComponentPtr_t& cc1,
		    const ConnectedComponentPtr_t& cc2);

      /// Merge two connected components
      /// \param cc1 the connected component to merge into
      /// \param the connected components to merge into cc1.
      void merge (const ConnectedComponentPtr_t& cc1,
		  ConnectedComponent::RawPtrs_t& ccs);

      const DistancePtr_t distance_;
      ConnectedComponents_t connectedComponents_;
      Nodes_t nodes_;
      Edges_t edges_;
      NodePtr_t initNode_;
      NodeVector_t goalNodes_;
      NearestNeighborPtr_t nearestNeighbor_;
      RoadmapWkPtr_t weak_;
    }; // class Roadmap
    std::ostream& operator<< (std::ostream& os, const Roadmap& r);
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_ROADMAP_HH
