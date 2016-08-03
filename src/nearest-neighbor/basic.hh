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

#ifndef HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH
# define HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/nearest-neighbor.hh>

namespace hpp {
  namespace core {
    namespace nearestNeighbor {
    /// Optimization of the nearest neighbor search
    class Basic : public NearestNeighbor
    {
    public:

      Basic(const DistancePtr_t& distance) : distance_ (distance)
      {
      }

      ~Basic()
      {
      }

      virtual void clear ()
      {
      }

      void addNode (const NodePtr_t&)
      {
      }

      virtual NodePtr_t search (const NodePtr_t& node,
             const ConnectedComponentPtr_t&
        connectedComponent,
             value_type& distance);

      virtual NodePtr_t search (const ConfigurationPtr_t& configuration,
			       const ConnectedComponentPtr_t&
				connectedComponent,
             value_type& distance, bool reverse = false);

      virtual Nodes_t KnearestSearch (const ConfigurationPtr_t& configuration,
                                      const ConnectedComponentPtr_t&
                                        connectedComponent,
                                      const std::size_t K,
                                      value_type& distance);

      virtual Nodes_t KnearestSearch (const NodePtr_t& node,
                                      const ConnectedComponentPtr_t&
                                        connectedComponent,
                                      const std::size_t K,
                                      value_type& distance);

      virtual void merge (ConnectedComponentPtr_t, ConnectedComponentPtr_t)
      {
      }

      // Get distance function
      virtual DistancePtr_t distance () const
      {
	return distance_;
      }

    private:
      const DistancePtr_t distance_;
    }; // class Basic
    } // namespace nearestNeighbor
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_NEAREST_NEIGHBOR_BASIC_HH
