// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE hermite - paths
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/hermite.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/object-factory.hh>

#define TOSTR(x) \
  static_cast<std::ostringstream&>((std::ostringstream() << x)).str()

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;

using namespace hpp::core;

hpp::model::ObjectFactory objectFactory;

DevicePtr_t createRobot() {
  DevicePtr_t robot = Device::create("test");

  const std::string& name = robot->name();
  coal::Transform3s mat;
  mat.setIdentity();
  JointPtr_t joint;
  std::string jointName = name + "_xy";
  // Translation along x
  coal::Matrix3f permutation;
  joint = objectFactory.createJointTranslation2(mat);
  joint->name(jointName);

  joint->isBounded(0, 1);
  joint->lowerBound(0, -4);
  joint->upperBound(0, +4);
  joint->isBounded(1, 1);
  joint->lowerBound(1, -4);
  joint->upperBound(1, +4);

  robot->rootJoint(joint);
  return robot;
}

template <int Cols>
struct Bezier {
  typedef Eigen::Matrix<value_type, Eigen::Dynamic, Cols> Coefficients_t;

  vector_t operator()(const value_type& t) const {
    assert(0 <= t && t <= 1);
    return run<0>(coeffs, t);
  }

  vector_t velocity(const value_type& t) const {
    assert(0 <= t && t <= 1);
    return derivative<0>(coeffs, t);
  }

  template <int CStart, typename Derived>
  static vector_t run(const Eigen::MatrixBase<Derived>& coefs,
                      const value_type& t) {
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime >= (Cols + CStart),
                        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    typedef Bezier<Cols - 1> NextBezier;
    return (1 - t) * NextBezier::template run<CStart>(coefs.derived(), t) +
           t * NextBezier::template run<CStart + 1>(coefs.derived(), t);
  }

  template <int CStart, typename Derived>
  static vector_t derivative(const Eigen::MatrixBase<Derived>& coefs,
                             const value_type& t) {
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime >= (Cols + CStart),
                        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    typedef Bezier<Cols - 1> NextBezier;
    return (1 - t) *
               NextBezier::template derivative<CStart>(coefs.derived(), t) -
           NextBezier::template run<CStart>(coefs.derived(), t) +
           t * NextBezier::template derivative<CStart + 1>(coefs.derived(), t) +
           NextBezier::template run<CStart + 1>(coefs.derived(), t);
  }

  Coefficients_t coeffs;
};

template <>
struct Bezier<1> {
  template <int Col, typename Derived>
  static typename Derived::ConstColXpr run(
      const Eigen::MatrixBase<Derived>& coefs, const value_type&) {
    // EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 1,
    // THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    return coefs.derived().col(Col);
  }

  template <int Col, typename Derived>
  static typename vector_t::ConstantReturnType derivative(
      const Eigen::MatrixBase<Derived>& coefs, const value_type&) {
    return vector_t::Zero(coefs.rows());
  }
};

Bezier<4> bezierCurveFromHermite(vectorIn_t x0, vectorIn_t x1, vectorIn_t v0,
                                 vectorIn_t v1) {
  Bezier<4> bezier;
  bezier.coeffs.resize(x0.rows(), 4);

  bezier.coeffs.col(0) = x0.transpose();
  bezier.coeffs.col(1) = (x0 + v0 / 3).transpose();
  bezier.coeffs.col(2) = (x1 - v1 / 3).transpose();
  bezier.coeffs.col(3) = x1.transpose();

  std::cout << "P0: " << x0.transpose() << '\n'
            << "P1: " << (x0 + v0 / 3).transpose() << '\n'
            << "P2: " << (x1 - v1 / 3).transpose() << '\n'
            << "P3: " << x1.transpose() << std::endl;
  return bezier;
}

BOOST_AUTO_TEST_CASE(hermitePath) {
  DevicePtr_t dev = createRobot();
  BOOST_REQUIRE(dev);
  ProblemPtr_t problem = Problem::create(dev);
  steeringMethod::HermitePtr_t hermiteSM =
      steeringMethod::Hermite::create(problem);

  Configuration_t q0(dev->configSize());
  q0 << -1, -1;
  Configuration_t q2(dev->configSize());
  q2 << 1, 1;

  Configuration_t lv(dev->configSize());
  lv << 2, 2;

  Configuration_t v0(dev->configSize());
  v0 << 3, 2.5;
  Configuration_t v2(dev->configSize());
  v2 << 1, 1.5;

  Configuration_t q1(dev->configSize());
  Configuration_t v1(dev->configSize());

  path::HermitePtr_t p =
      HPP_DYNAMIC_PTR_CAST(path::Hermite, (*hermiteSM)(q0, q2));
  p->v0(v0);
  p->v1(v2);
  p->computeHermiteLength();

  Bezier<4> bezier = bezierCurveFromHermite(q0, q2, v0, v2);

  bool s;
  std::cout << std::endl;
  std::cout << (*p) << std::endl;
  std::cout << p->v0().transpose() << std::endl;
  std::cout << p->v1().transpose() << std::endl;
  std::cout << std::endl;
  std::cout << (*p)(0, s).transpose() << std::endl;
  std::cout << (*p)(0.5, s).transpose() << std::endl;
  std::cout << bezier(0.5).transpose() << std::endl;
  std::cout << p->velocity(0.5).transpose() << std::endl;
  std::cout << bezier.velocity(0.5).transpose() << std::endl;
  std::cout << p->velocity(0).transpose() << std::endl;
  std::cout << bezier.velocity(0).transpose() << std::endl;
  std::cout << p->velocity(1).transpose() << std::endl;
  std::cout << bezier.velocity(1).transpose() << std::endl;
  std::cout << (*p)(1, s).transpose() << std::endl;
}
