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

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/k-d-tree.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (NearestNeighbor);
    typedef boost::shared_ptr <NearestNeighbor> NearestNeighborPtr_t;

    /// Roadmap built by random path planning methods
    /// Nodes are configurations, paths are collision-free paths.
    class HPP_CORE_DLLAPI Roadmap {
    public:
      /// Return shared pointer to new instance.
      static RoadmapPtr_t create (const DistancePtr_t& distance, const DevicePtr_t& robot);
      void clear ();
      /// Add a node with given configuration
      /// \param config configuration
      /// \param connectedComponent Connected component the node belongs to.
      ///        nil pointer means the whole roadmap.
      ///
      /// If configuration is alread in the connected component, return the node
      /// containing the configuration. Otherwise, create a new node with given
      /// connected component (or new connected component if nil pointer).
      NodePtr_t addNode (const ConfigurationPtr_t& config,
			 ConnectedComponentPtr_t connectedComponent =
			 ConnectedComponentPtr_t());

      /// Get nearest node to a configuration in a connected component.
      /// \param configuration configuration
      /// \param connectedComponent the connected component
      /// \retval distance to the nearest node.
      /// If pointer to connectedComponent is nil, looks into all connected
      /// components.
      NodePtr_t nearestNode (const ConfigurationPtr_t& configuration,
			     const ConnectedComponentPtr_t& connectedComponent,
			     value_type& minDistance);

      /// Add a node and an edge
      /// \param from node from which the edge starts,
      /// \param to configuration to which the edge stops
      /// \param path path between both configurations
      /// \return node containing configuration <c>to</c>.
      /// Add the symmetric edge with reverse path.
      NodePtr_t addNodeAndEdge (const NodePtr_t from,
				const ConfigurationPtr_t& to,
				const PathPtr_t path);

      /// Add a goal configuration
      /// \param config configuration
      /// If configuration is already in the roadmap, tag corresponding node
      /// as goal node. Otherwise create a new node.
      void addGoalNode (const ConfigurationPtr_t& config);

      void resetGoalNodes ()
      {
	goalNodes_.clear ();
      }

      void initNode (const ConfigurationPtr_t& config)
      {
	initNode_ = addNode (config);
      }

      virtual ~Roadmap ();
      /// Find a path between initial and goal configurations
      PathPtr_t findPath () const;
      const Nodes_t& nodes () const
      {
	return nodes_;
      }
      const Edges_t& edges () const
      {
	return edges_;
      }
      const ConnectedComponents_t& connectedComponents () const
      {
	return connectedComponents_;
      }
      NodePtr_t initNode () const
      {
	return initNode_;
      }
      const Nodes_t& goalNodes () const
      {
	return goalNodes_;
      }
      /// \name Distance used for nearest neighbor search
      /// \{
      /// Get distance function
      const DistancePtr_t& distance () const;
      /// \}
      /// Add an edge between two nodes.
      EdgePtr_t addEdge (const NodePtr_t& n1, const NodePtr_t& n2,
			 const PathPtr_t& path);

    protected:
      /// Constructor
      /// \param distance distance function for nearest neighbor computations
      Roadmap (const DistancePtr_t& distance, const DevicePtr_t& robot);

      /// Add a new connected component in the roadmap.
      /// \param node node pointing to the connected component.
      /// \note The node is added in the connected component.
      void addConnectedComponent (const NodePtr_t& node);

    private:
      typedef std::map <ConnectedComponentPtr_t, NearestNeighborPtr_t>
	NearetNeighborMap_t;
      const DistancePtr_t& distance_;
      ConnectedComponents_t connectedComponents_;
      Nodes_t nodes_;
      Edges_t edges_;
      NodePtr_t initNode_;
      Nodes_t goalNodes_;
      // use KDTree instead of NearestNeighbor 
      //NearetNeighborMap_t nearestNeighbor_;
      KDTree kdTree_;

    }; // class Roadmap
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_ROADMAP_HH
