//
// Copyright (c) 2020 CNRS
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

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/serialization.hh>
#include <hpp/pinocchio/serialization.hh>

#include <hpp/core/connected-component.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>

namespace hpp {
namespace core {

template <typename Archive>
inline void Node::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  if (!Archive::is_saving::value && !configuration_)
    configuration_.reset (new Configuration_t);
  Configuration_t& config (*configuration_);
  serialization::remove_duplicate::serialize_vector(ar, "config", config, version);
  ar & BOOST_SERIALIZATION_NVP(outEdges_);
  ar & BOOST_SERIALIZATION_NVP(inEdges_);
  ar & BOOST_SERIALIZATION_NVP(connectedComponent_);
}
HPP_SERIALIZATION_IMPLEMENT(Node);

template <typename Archive>
inline void Edge::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & BOOST_SERIALIZATION_NVP(n1_);
  ar & BOOST_SERIALIZATION_NVP(n2_);
  ar & BOOST_SERIALIZATION_NVP(path_);
}
HPP_SERIALIZATION_IMPLEMENT(Edge);

template <typename Archive>
inline void ConnectedComponent::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & BOOST_SERIALIZATION_NVP(nodes_);
  ar & BOOST_SERIALIZATION_NVP(reachableFrom_);
  ar & BOOST_SERIALIZATION_NVP(reachableTo_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(ConnectedComponent);

template <typename Archive>
inline void Roadmap::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & boost::serialization::make_nvp("distance_", const_cast<DistancePtr_t&>(distance_));
  ar & BOOST_SERIALIZATION_NVP(connectedComponents_);
  ar & BOOST_SERIALIZATION_NVP(nodes_);
  ar & BOOST_SERIALIZATION_NVP(edges_);
  ar & BOOST_SERIALIZATION_NVP(initNode_);
  ar & BOOST_SERIALIZATION_NVP(goalNodes_);
  ar & BOOST_SERIALIZATION_NVP(nearestNeighbor_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(Roadmap);

} //   namespace core
} // namespace hpp
