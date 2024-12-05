// Copyright (c) 2019 CNRS
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

#include <coal/collision.h>

#include <hpp/core/obstacle-user.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/exception-factory.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace hpp {
namespace core {
using ::pinocchio::toFclTransform3f;
bool ObstacleUser::collide(const CollisionPairs_t& pairs,
                           CollisionRequests_t& reqs,
                           coal::CollisionResult& res, std::size_t& i,
                           pinocchio::DeviceData& data) {
  for (i = 0; i < pairs.size(); ++i) {
    res.clear();
    if (pairs[i].collide(data, reqs[i], res) != 0) return true;
  }
  return false;
}

void ObstacleUser::addObstacle(const CollisionObjectConstPtr_t& object) {
  for (size_type j = 0; j < robot_->nbJoints(); ++j) {
    JointPtr_t joint = robot_->jointAt(j);
    addObstacleToJoint(object, joint, false);
  }
}

void ObstacleUser::addObstacleToJoint(const CollisionObjectConstPtr_t& object,
                                      const JointPtr_t& joint,
                                      const bool includeChildren) {
  BodyPtr_t body = joint->linkedBody();
  if (body) {
    for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
      // TODO: check the objects are not in same joint
      cPairs_.push_back(CollisionPair_t(body->innerObjectAt(o), object));
      cRequests_.push_back(defaultRequest_);
    }
  }
  if (includeChildren) {
    for (std::size_t i = 0; i < joint->numberChildJoints(); ++i) {
      addObstacleToJoint(object, joint->childJoint(i), includeChildren);
    }
  }
}

struct CollisionPairComparision {
  CollisionPair_t a;
  CollisionPairComparision(const CollisionPair_t& p) : a(p) {}
  bool operator()(const CollisionPair_t& b) {
    return (&(a.first->pinocchio()) == &(b.first->pinocchio())) &&
           (&(a.second->pinocchio()) == &(b.second->pinocchio()));
  }
};

void ObstacleUser::removeObstacleFromJoint(
    const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle) {
  BodyPtr_t body = joint->linkedBody();
  if (body) {
    for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
      CollisionPair_t colPair(body->innerObjectAt(o), obstacle);
      CollisionPairComparision compare(colPair);
      std::size_t nbDelPairs = 0;
      for (std::size_t i = 0; i < cPairs_.size();) {
        if (compare(cPairs_[i])) {
          cPairs_.erase(cPairs_.begin() + i);
          cRequests_.erase(cRequests_.begin() + i);
          ++nbDelPairs;
        } else
          ++i;
      }
      if (nbDelPairs == 0) {
        std::ostringstream oss;
        oss << "ObstacleUser::removeObstacleFromJoint: obstacle \""
            << obstacle->name()
            << "\" is not registered as obstacle for joint \"" << joint->name()
            << "\".";
        throw std::runtime_error(oss.str());
      } else if (nbDelPairs >= 2) {
        hppDout(error, "obstacle " << obstacle->name() << " was registered "
                                   << nbDelPairs
                                   << " times as obstacle for joint "
                                   << joint->name() << ".");
      }
    }
  }
}

void ObstacleUser::filterCollisionPairs(
    const RelativeMotion::matrix_type& matrix) {
  // Loop over collision pairs and remove disabled ones.
  pinocchio::JointIndex j1, j2;
  coal::CollisionResult unused;
  for (std::size_t i = 0; i < cPairs_.size();) {
    const CollisionPair_t& pair = cPairs_[i];

    j1 = pair.first->jointIndex();
    j2 = pair.second->jointIndex();

    switch (matrix(j1, j2)) {
      case RelativeMotion::Parameterized:
        hppDout(info, "Parameterized collision pairs between "
                          << pair.first->name() << " and "
                          << pair.second->name());
        pPairs_.push_back(pair);
        pRequests_.push_back(cRequests_[i]);
        cPairs_.erase(cPairs_.begin() + i);
        cRequests_.erase(cRequests_.begin() + i);
        break;
      case RelativeMotion::Constrained:
        hppDout(info, "Disabling collision between " << pair.first->name()
                                                     << " and "
                                                     << pair.second->name());
        if (pair.collide(cRequests_[i], unused) != 0) {
          hppDout(warning,
                  "Disabling collision detection between two "
                  "bodies in collision.");
        }
        dPairs_.push_back(pair);
        dRequests_.push_back(cRequests_[i]);
        cPairs_.erase(cPairs_.begin() + i);
        cRequests_.erase(cRequests_.begin() + i);
        break;
      case RelativeMotion::Unconstrained:
        ++i;
        break;
      default:
        hppDout(warning, "RelativeMotionType not understood");
        ++i;
        break;
    }
  }
}

void ObstacleUser::setSecurityMargins(const matrix_t& securityMatrix) {
  if (securityMatrix.rows() != robot_->nbJoints() + 1 ||
      securityMatrix.cols() != robot_->nbJoints() + 1) {
    HPP_THROW(std::invalid_argument,
              "Wrong size of security margin matrix."
              " Expected "
                  << robot_->nbJoints() + 1 << 'x' << robot_->nbJoints() + 1
                  << ". Got " << securityMatrix.rows() << 'x'
                  << securityMatrix.cols());
  }
  pinocchio::JointIndex j1, j2;
  coal::CollisionResult unused;
  for (std::size_t i = 0; i < cPairs_.size(); ++i) {
    const CollisionPair_t& pair = cPairs_[i];
    j1 = pair.first->jointIndex();
    j2 = pair.second->jointIndex();
    cRequests_[i].security_margin = securityMatrix(j1, j2);
  }
  for (std::size_t i = 0; i < pPairs_.size(); ++i) {
    const CollisionPair_t& pair = pPairs_[i];
    j1 = pair.first->jointIndex();
    j2 = pair.second->jointIndex();
    pRequests_[i].security_margin = securityMatrix(j1, j2);
  }
  for (std::size_t i = 0; i < dPairs_.size(); ++i) {
    const CollisionPair_t& pair = dPairs_[i];
    j1 = pair.first->jointIndex();
    j2 = pair.second->jointIndex();
    dRequests_[i].security_margin = securityMatrix(j1, j2);
  }
}

void ObstacleUser::setSecurityMarginBetweenBodies(const std::string& body_a,
                                                  const std::string& body_b,
                                                  const value_type& margin) {
  auto setMargin = [&body_a, &body_b, &margin](const CollisionPairs_t& pairs,
                                               CollisionRequests_t& requests) {
    for (std::size_t i = 0; i < pairs.size(); ++i) {
      const CollisionPair_t& pair = pairs[i];
      if ((pair.first->name() == body_a && pair.second->name() == body_b) ||
          (pair.first->name() == body_b && pair.second->name() == body_a)) {
        requests[i].security_margin = margin;
        return true;
      }
    }
    return false;
  };
  if (setMargin(cPairs_, cRequests_) || setMargin(pPairs_, pRequests_) ||
      setMargin(dPairs_, dRequests_))
    return;
  throw std::invalid_argument(
      "Could not find a collision pair between body"
      " " +
      body_a + " and " + body_b);
}

void ObstacleUser::addRobotCollisionPairs() {
  const pinocchio::GeomModel& model = robot_->geomModel();
  const pinocchio::GeomData& data = robot_->geomData();

  for (std::size_t i = 0; i < model.collisionPairs.size(); ++i)
    if (data.activeCollisionPairs[i]) {
      CollisionObjectConstPtr_t o1(new pinocchio::CollisionObject(
          robot_, model.collisionPairs[i].first));
      CollisionObjectConstPtr_t o2(new pinocchio::CollisionObject(
          robot_, model.collisionPairs[i].second));
      cPairs_.push_back(CollisionPair_t(o1, o2));
      cRequests_.push_back(defaultRequest_);
    }
}

void ObstacleUser::setRequests(const coal::CollisionRequest& r) {
  for (std::size_t i = 0; i < cRequests_.size(); ++i) cRequests_[i] = r;
  for (std::size_t i = 0; i < pRequests_.size(); ++i) pRequests_[i] = r;
  for (std::size_t i = 0; i < dRequests_.size(); ++i) dRequests_[i] = r;
}
}  // namespace core
}  // namespace hpp
