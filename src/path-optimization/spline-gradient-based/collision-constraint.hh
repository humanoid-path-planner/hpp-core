//
// Copyright (c) 2015 CNRS
// Author: Mylene Campana, Joseph Mirabel
//
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH

#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/util.hh>
#include <hpp/util/exception-factory.hh>
#include <pinocchio/multibody/liegroup/liegroup.hpp>

namespace hpp {
namespace core {
namespace pathOptimization {

template <typename SplinePtr_t>
class CollisionFunction : public DifferentiableFunction {
 public:
  typedef shared_ptr<CollisionFunction> Ptr_t;
  virtual ~CollisionFunction() {}
  static Ptr_t create(const DevicePtr_t& robot, const SplinePtr_t& freeSpline,
                      const SplinePtr_t& collSpline,
                      const PathValidationReportPtr_t& report) {
    CollisionFunction* ptr =
        new CollisionFunction(robot, freeSpline, collSpline, report);
    return Ptr_t(ptr);
  }

  void updateConstraint(const Configuration_t& q) {
    fcl::CollisionResult result = checkCollision(q, true);

    if (result.numContacts() == 1) {  // Update qColl_
      qColl_ = q;
      contactPoint_ = result.getContact(0).pos;
      hppDout(info, "contact point = " << contactPoint_.transpose());
    } else {  // Update qFree_
      qFree_ = q;
    }

    computeJacobian();
  }

  Configuration_t qFree_, qColl_;

 protected:
  CollisionFunction(const DevicePtr_t& robot, const SplinePtr_t& freeSpline,
                    const SplinePtr_t& collSpline,
                    const PathValidationReportPtr_t& report)
      : DifferentiableFunction(robot->configSize(), robot->numberDof(),
                               LiegroupSpace::R1(), ""),
        qFree_(),
        qColl_(),
        robot_(robot),
        object1_(),
        object2_(),
        J_(),
        difference_(robot->numberDof()) {
    const value_type& tColl = report->parameter;
    bool success;
    qColl_ = collSpline->eval(tColl, success);
    assert(success);

    CollisionValidationReportPtr_t collisionReport = (HPP_DYNAMIC_PTR_CAST(
        CollisionValidationReport, report->configurationReport));
    if (!collisionReport) {
      std::stringstream ss;
      ss << "Expected a CollisionValidationReport. Got a "
         << *report->configurationReport;
      throw std::logic_error(ss.str());
    }
    object1_ = collisionReport->object1;
    object2_ = collisionReport->object2;

    hppDout(info, "obj1 = " << object1_->name()
                            << " and obj2 = " << object2_->name());
    hppDout(info, "qColl = " << pinocchio::displayConfig(qColl_));

    // Backtrack collision in previous path (x0) to create constraint
    value_type tFree = tColl * freeSpline->length() / collSpline->length();
    tFree = std::min(tFree,
                     freeSpline->length());  // can be slightly above freeSpline
                                             // length due to round-offs

    qFree_ = freeSpline->eval(tFree, success);
    assert(success);
    hppDout(info, "qFree = " << pinocchio::displayConfig(qFree_));
    // Compute contact point in configuration qColl
    const fcl::CollisionResult& result(collisionReport->result);
    if (result.numContacts() < 1) {
      abort();
    }
    assert(result.numContacts() >= 1);
    contactPoint_ = result.getContact(0).pos;
    hppDout(info, "contact point = " << contactPoint_.transpose());
    computeJacobian();
  }

  fcl::CollisionResult checkCollision(const Configuration_t& q,
                                      bool enableContact) {
    pinocchio::DeviceSync device(robot_);
    device.currentConfiguration(q);
    device.computeForwardKinematics(pinocchio::JOINT_POSITION);
    device.updateGeometryPlacements();
    fcl::CollisionResult result;
    fcl::CollisionRequestFlag flag =
        enableContact ? fcl::CONTACT : fcl::NO_REQUEST;
    fcl::CollisionRequest collisionRequest(flag, 1);
    using ::pinocchio::toFclTransform3f;
    fcl::collide(object1_->geometry().get(),
                 toFclTransform3f(object1_->getTransform(device.d())),
                 object2_->geometry().get(),
                 toFclTransform3f(object2_->getTransform(device.d())),
                 collisionRequest, result);
    return result;
  }

  void computeJacobian() {
    static const matrix3_t I3(matrix3_t::Identity());

    DifferentiableFunctionPtr_t f;
    vector3_t u;

    pinocchio::DeviceSync device(robot_);
    device.currentConfiguration(qColl_);
    device.computeForwardKinematics(pinocchio::JOINT_POSITION |
                                    pinocchio::JACOBIAN);

    if (!object2_->joint() || object2_->joint()->index() == 0) {
      object1_.swap(object2_);
    }

    JointConstPtr_t joint1 = object1_->joint();
    JointConstPtr_t joint2 = object2_->joint();
    assert(joint2 && joint2->index() > 0);
    Transform3f M2(joint2->currentTransformation(device.d()));
    vector3_t x2_J2(M2.actInv(contactPoint_));

    if (joint1 && joint1->index() > 0) {  // object1 = body part
      Transform3f M1(joint1->currentTransformation(device.d()));
      // Position of contact point in each object local frame
      vector3_t x1_J1 = M1.actInv(contactPoint_);
      // Compute contact points in configuration qFree
      device.currentConfiguration(qFree_);
      device.computeForwardKinematics(pinocchio::JOINT_POSITION);
      M2 = joint2->currentTransformation(device.d());
      M1 = joint1->currentTransformation(device.d());
      device.unlock();
      // Position of x1 in local frame of joint2
      vector3_t x1_J2(M2.actInv(M1.act(x1_J1)));
      hppDout(info, "x2 in J2 = " << x2_J2.transpose());
      hppDout(info, "x1 in J2 = " << x1_J2.transpose());

      u = (x1_J2 - x2_J2).normalized();
      f = constraints::RelativePosition::create("", robot_, joint1, joint2,
                                                Transform3f(I3, x1_J1),
                                                Transform3f(I3, x2_J2));
    } else {  // object1 = fixed obstacle and has no joint
      vector3_t x1_J1(contactPoint_);
      // Compute contact points in configuration qFree
      device.currentConfiguration(qFree_);
      device.computeForwardKinematics(pinocchio::JOINT_POSITION);
      Transform3f M2(joint2->currentTransformation(device.d()));
      device.unlock();
      // position of x2 in global frame
      vector3_t x2_J1(M2.act(x2_J2));
      hppDout(info, "x2 in J1 = " << x2_J1.transpose());

      u = (x2_J1 - x2_J2).normalized();
      f = constraints::Position::create(
          "", robot_, joint2, Transform3f(I3, x2_J2), Transform3f(I3, x1_J1));
    }
    matrix_t Jpos(f->outputSize(), f->inputDerivativeSize());
    f->jacobian(Jpos, qFree_);
    J_ = u.transpose() * Jpos;
    assert(J_.rows() == 1);
  }

  virtual void impl_compute(LiegroupElementRef result,
                            vectorIn_t argument) const {
    pinocchio::difference<pinocchio::DefaultLieGroupMap>(robot_, argument,
                                                         qFree_, difference_);
    result.vector() = J_ * difference_;
    result.check();
  }
  virtual void impl_jacobian(matrixOut_t jacobian, vectorIn_t) const {
    jacobian = J_;
  }

 private:
  DevicePtr_t robot_;
  CollisionObjectConstPtr_t object1_, object2_;
  matrix_t J_;
  vector3_t contactPoint_;
  mutable vector_t difference_;
};  // class CollisionFunction
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
