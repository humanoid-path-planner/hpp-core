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

# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// Optimization of the nearest neighbor search
    class NearestNeighbor
    {
    public:
      virtual void clear () = 0;
      virtual void addNode (const NodePtr_t& node) = 0;

      /**
       * @brief search Return the closest node of the given configuration
       * @param configuration
       * @param connectedComponent
       * @param distance
       * @param reverse if true, compute distance from given configuration to nodes in roadmap,
       *  if false from nodes in roadmap to given configuration
       * @return
       */
      virtual NodePtr_t search (const ConfigurationPtr_t& configuration,
			       const ConnectedComponentPtr_t&
				connectedComponent,
             value_type& distance,bool reverse = false) = 0;


      virtual NodePtr_t search (const NodePtr_t& node,
			       const ConnectedComponentPtr_t&
				connectedComponent,
			       value_type& distance) = 0;

      /// \param[out] distance to the Kth closest neighbor
      /// \return the K nearest neighbors
      virtual Nodes_t KnearestSearch (const ConfigurationPtr_t& configuration,
			              const ConnectedComponentPtr_t&
                                        connectedComponent,
                                      const std::size_t K,
			              value_type& distance) = 0;

      /// \param[out] distance to the Kth closest neighbor
      /// \return the K nearest neighbors
      virtual Nodes_t KnearestSearch (const NodePtr_t& node,
			              const ConnectedComponentPtr_t&
                                        connectedComponent,
                                      const std::size_t K,
			              value_type& distance) = 0;

      // merge two connected components in the whole tree
      virtual void merge (ConnectedComponentPtr_t cc1,
			  ConnectedComponentPtr_t cc2) = 0;

      // Get distance function
      virtual DistancePtr_t distance () const = 0;

    }; // class NearestNeighbor
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_NEAREST_NEIGHBOR_HH
