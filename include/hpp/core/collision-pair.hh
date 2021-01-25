//
// Copyright (c) 2021 CNRS
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

#ifndef HPP_CORE_COLLISION_PAIR_HH
# define HPP_CORE_COLLISION_PAIR_HH

#include <hpp/fcl/collision.h>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {

struct CollisionPair {
  CollisionObjectConstPtr_t first;
  CollisionObjectConstPtr_t second;
  fcl::ComputeCollision computeCollision;

  inline CollisionPair (CollisionObjectConstPtr_t f, CollisionObjectConstPtr_t s)
    : first(f), second(s), computeCollision(f->geometry().get(), s->geometry().get())
  {}

  inline auto collide(fcl::CollisionRequest& request, fcl::CollisionResult& result) const
    //decltype(computeCollision(tf1,tf2,request,result))
  {
    assert(!first ->getTransform().translation().hasNaN());
    assert(!first ->getTransform().rotation   ().hasNaN());
    assert(!second->getTransform().translation().hasNaN());
    assert(!second->getTransform().rotation   ().hasNaN());
    return computeCollision(
        first ->getFclTransform(),
        second->getFclTransform(),
        request,result);
  }

  inline auto collide(const pinocchio::DeviceData& d,
      fcl::CollisionRequest& request, fcl::CollisionResult& result) const
    //decltype(computeCollision(tf1,tf2,request,result))
  {
    using ::pinocchio::toFclTransform3f;
    assert(!first ->getTransform(d).translation().hasNaN());
    assert(!first ->getTransform(d).rotation   ().hasNaN());
    assert(!second->getTransform(d).translation().hasNaN());
    assert(!second->getTransform(d).rotation   ().hasNaN());
    return computeCollision(
        toFclTransform3f(first ->getTransform (d)),
        toFclTransform3f(second->getTransform (d)),
        request,result);
  }
};

} // namespace core
} // namespace hpp
#endif // HPP_CORE_COLLISION_PAIR_HH
