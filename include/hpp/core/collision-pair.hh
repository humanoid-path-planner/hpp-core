//
// Copyright (c) 2021 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_COLLISION_PAIR_HH
#define HPP_CORE_COLLISION_PAIR_HH

#include <coal/collision.h>

#include <hpp/core/fwd.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device-data.hh>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace hpp {
namespace core {

typedef std::vector<coal::CollisionRequest> CollisionRequests_t;

struct CollisionPair {
  CollisionObjectConstPtr_t first;
  CollisionObjectConstPtr_t second;
  coal::ComputeCollision computeCollision;

  inline CollisionPair(CollisionObjectConstPtr_t f, CollisionObjectConstPtr_t s)
      : first(f),
        second(s),
        computeCollision(f->geometry().get(), s->geometry().get()) {}

  inline auto collide(coal::CollisionRequest& request,
                      coal::CollisionResult& result) const
  // decltype(computeCollision(tf1,tf2,request,result))
  {
    assert(!first->getTransform().translation().hasNaN());
    assert(!first->getTransform().rotation().hasNaN());
    assert(!second->getTransform().translation().hasNaN());
    assert(!second->getTransform().rotation().hasNaN());
    return computeCollision(first->getFclTransform(), second->getFclTransform(),
                            request, result);
  }

  inline auto collide(const pinocchio::DeviceData& d,
                      coal::CollisionRequest& request,
                      coal::CollisionResult& result) const
  // decltype(computeCollision(tf1,tf2,request,result))
  {
    using ::pinocchio::toFclTransform3f;
    assert(!first->getTransform(d).translation().hasNaN());
    assert(!first->getTransform(d).rotation().hasNaN());
    assert(!second->getTransform(d).translation().hasNaN());
    assert(!second->getTransform(d).rotation().hasNaN());
    return computeCollision(toFclTransform3f(first->getTransform(d)),
                            toFclTransform3f(second->getTransform(d)), request,
                            result);
  }
};

}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_COLLISION_PAIR_HH
