//
// Copyright (c) 2016 CNRS
// Authors: Florent Lamiraux
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

#define BOOST_TEST_MODULE path_extraction

#include <cmath>
#include <coal/math/transform.h>

#include <boost/test/included/unit_test.hpp>
#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/time-parameterization/polynomial.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/fwd.hpp>

BOOST_AUTO_TEST_SUITE(test_hpp_core)

using hpp::constraints::ComparisonTypes_t;
using hpp::constraints::DifferentiableFunction;
using hpp::constraints::DifferentiableFunctionPtr_t;
using hpp::constraints::Identity;
using hpp::constraints::Implicit;
using hpp::constraints::ImplicitPtr_t;
using hpp::core::ConstraintSet;
using hpp::core::ConstraintSetPtr_t;
using hpp::core::ConfigProjector;
using hpp::core::ConfigProjectorPtr_t;
using hpp::core::InterpolatedPath;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::ConfigurationOut_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointConstPtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::LiegroupSpace;

using namespace hpp::core;

HPP_PREDEF_CLASS(LocalPath);
typedef std::shared_ptr<LocalPath> LocalPathPtr_t;

// Linear interpolation between two points
class LocalPath : public Path {
  public:
  virtual ~LocalPath() {}
  virtual bool impl_compute(ConfigurationOut_t configuration,
                            value_type t) const {
    value_type alpha(t - timeRange().first /
                             (timeRange().second - timeRange().first));
    configuration = (1 - alpha) * q1_ + alpha * q2_;
    return true;
  }

  // Create a linear path interpolation between t_min and t_max
  static LocalPathPtr_t create(const interval_t& interval) {
    LocalPath* ptr(new LocalPath(interval));
    LocalPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  // Create a linear interpolation between q1 and q2
  static LocalPathPtr_t create(const interval_t& interval, ConfigurationIn_t q1,
			       ConfigurationIn_t q2, ConstraintSetPtr_t constraints) {
    LocalPath* ptr(new LocalPath(interval, q1, q2, constraints));
    LocalPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }
  static LocalPathPtr_t createCopy(const LocalPathPtr_t& other) {
    LocalPath* ptr(new LocalPath(*other));
    LocalPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  virtual Configuration_t initial() const { return q1_; }

  virtual Configuration_t end() const { return q2_; }

  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  virtual PathPtr_t copy(const ConstraintSetPtr_t&) const {
    return createCopy(weak_.lock());
  }

  virtual std::ostream& print(std::ostream& os) const {
    os << "LocalPath between " << q1_[0] << " and " << q2_[0];
    return os;
  }

 protected:
  LocalPath(const interval_t& interval) : Path(interval, 1, 1), q1_(1), q2_(1) {
    q1_[0] = interval.first;
    q2_[0] = interval.second;
  }

  LocalPath(const interval_t& interval, ConfigurationIn_t q1, ConfigurationIn_t q2,
	    const ConstraintSetPtr_t& constraints) :
    Path(interval, q1.size(), q1.size(), constraints), q1_(q1), q2_(q2) {
  }

  LocalPath(const LocalPath& other)
      : Path(other), q1_(other.q1_), q2_(other.q2_) {}

  void init(LocalPathWkPtr_t wkPtr) {
    Path::init(wkPtr.lock());
    weak_ = wkPtr;
  }

 private:
  Configuration_t q1_, q2_;
  LocalPathWkPtr_t weak_;
};

BOOST_AUTO_TEST_CASE(path_extraction_1) {
  PathVectorPtr_t pv(PathVector::create(1, 1));
  // l0 = 0.52843097738664657
  interval_t inter0(std::make_pair(0, 0.52843097738664657));
  // L0 + l1 = 2.633158413063464
  interval_t inter1(std::make_pair(0, 2.1047274356768177));
  // l0 + l1 + l2 = 3.331454231080471
  interval_t inter2(std::make_pair(0.6751734883957865, 1.3734693064127934));
  // l0 + l1 + l2 + l3 = 4.079212904814735
  interval_t inter3(std::make_pair(0.83075838111776035, 1.578517054852024));
  LocalPathPtr_t p0(LocalPath::create(inter0));
  LocalPathPtr_t p1(LocalPath::create(inter1));
  LocalPathPtr_t p2(LocalPath::create(inter2));
  LocalPathPtr_t p3(LocalPath::create(inter3));
  pv->appendPath(p0);
  pv->appendPath(p1);
  pv->appendPath(p2);
  pv->appendPath(p3);

  value_type tmin(3.7487655421722721), tmax(4.0792129048147352);
  PathPtr_t extracted(pv->extract(std::make_pair(tmin, tmax)));
  BOOST_CHECK_EQUAL(extracted->timeRange().first, 0);
  BOOST_CHECK_CLOSE(extracted->timeRange().second, tmax - tmin,
                    std::numeric_limits<float>::epsilon());
  BOOST_CHECK_CLOSE(extracted->length(), tmax - tmin,
                    std::numeric_limits<float>::epsilon());

  extracted = pv->extract(std::make_pair(tmax, tmin));
  BOOST_CHECK_EQUAL(extracted->timeRange().first, 0);
  BOOST_CHECK_CLOSE(extracted->timeRange().second, tmax - tmin,
                    std::numeric_limits<float>::epsilon());
  BOOST_CHECK_CLOSE(extracted->length(), tmax - tmin,
                    std::numeric_limits<float>::epsilon());
}

BOOST_AUTO_TEST_CASE(path_extraction_2) {
  PathVectorPtr_t pv(PathVector::create(1, 1));
  // l0 = 0.52843097738664657
  interval_t inter0(std::make_pair(0, 0.52843097738664657));
  // L0 + l1 = 2.633158413063464
  interval_t inter1(std::make_pair(0, 2.1047274356768177));
  // l0 + l1 + l2 = 3.331454231080471
  interval_t inter2(std::make_pair(0.6751734883957865, 1.3734693064127934));
  // l0 + l1 + l2 + l3 = 4.079212904814735
  interval_t inter3(std::make_pair(0.83075838111776035, 1.578517054852024));
  LocalPathPtr_t p0(LocalPath::create(inter0));
  LocalPathPtr_t p1(LocalPath::create(inter1));
  LocalPathPtr_t p2(LocalPath::create(inter2));
  vector_t a(2);
  a << 0, 0.5;
  p2->timeParameterization(
      TimeParameterizationPtr_t(new timeParameterization::Polynomial(a)),
      interval_t(0, 2 * p2->length()));
  LocalPathPtr_t p3(LocalPath::create(inter3));
  pv->appendPath(p0);
  pv->appendPath(p1);
  pv->appendPath(p2);
  pv->appendPath(p3);

  value_type tmin(3.7487655421722721), tmax(4.0792129048147352);
  PathPtr_t extracted(pv->extract(std::make_pair(tmin, tmax)));
  BOOST_CHECK_EQUAL(extracted->timeRange().first, 0);
  BOOST_CHECK_CLOSE(extracted->timeRange().second, tmax - tmin,
                    std::numeric_limits<float>::epsilon());
  BOOST_CHECK_CLOSE(extracted->length(), tmax - tmin,
                    std::numeric_limits<float>::epsilon());

  extracted = pv->extract(std::make_pair(tmax, tmin));
  BOOST_CHECK_EQUAL(extracted->timeRange().first, 0);
  BOOST_CHECK_CLOSE(extracted->timeRange().second, tmax - tmin,
                    std::numeric_limits<float>::epsilon());
  BOOST_CHECK_CLOSE(extracted->length(), tmax - tmin,
                    std::numeric_limits<float>::epsilon());
}

class Rhs : public DifferentiableFunction {
public:
  static DifferentiableFunctionPtr_t create(value_type t0) {
    return DifferentiableFunctionPtr_t(new Rhs(t0));
  }
protected:
  Rhs(value_type t0) : DifferentiableFunction(1, 1, LiegroupSpace::R2(), "rhs"), t0_(t0) {
  }
  void impl_compute(LiegroupElementRef result,
		    vectorIn_t argument) const {
    value_type t = argument[0] + t0_;
    result.vector()[0] = cos(t);
    result.vector()[1] = sin(t);
  }
  void impl_jacobian(matrixOut_t jacobian, vectorIn_t arg) const {
    value_type t = arg[0];
    jacobian(0,0) = -sin(t);
    jacobian(1,0) =  cos(t);
  }
  value_type t0_;
}; // class Rhs

BOOST_AUTO_TEST_CASE(path_extraction_3) {
  // Build planar robot with 2 prismatic joints.
  DevicePtr_t robot(Device::create("xy"));
  const std::string urdfString(
      "<robot name=\"robot\">\n"
      "  <link name=\"base_link\"/>\n"
      "  <link name=\"lx\"/>\n"
      "  <link name=\"ly\"/>\n"
      "  <joint name=\"px\" type=\"prismatic\">\n"
      "    <limit effort=\"1.0\" lower=\"-2.0\" upper=\"2.0\" velocity=\"2.0\"/>\n"
      "    <axis xyz=\"1 0 0\"/>\n"
      "    <parent link=\"base_link\"/>\n"
      "    <child link=\"lx\"/>\n"
      "  </joint>\n"
      "  <joint name=\"py\" type=\"prismatic\">\n"
      "    <limit effort=\"1.0\" lower=\"-2.0\" upper=\"2.0\" velocity=\"2.0\"/>\n"
      "    <axis xyz=\"0 1 0\"/>"
      "    <parent link=\"lx\"/>\n"
      "    <child link=\"ly\"/>\n"
      "  </joint>\n"
      "</robot>");

  hpp::pinocchio::urdf::loadModelFromString(robot, 0, "robot", "anchor", urdfString, "");
  vector2_t q0; q0 << 1, 0;
  vector2_t q1; q1 << 0, 1;
  vector2_t q2; q2 << -1, 0;
  vector2_t q3; q3 << 0, -1;
  ConstraintSetPtr_t cs0(ConstraintSet::create(robot, "cs0"));
  ConstraintSetPtr_t cs1(ConstraintSet::create(robot, "cs1"));
  ConstraintSetPtr_t cs2(ConstraintSet::create(robot, "cs2"));
  ConstraintSetPtr_t cs3(ConstraintSet::create(robot, "cs3"));
  value_type threshold = 1e-4;
  ConfigProjectorPtr_t cp0(ConfigProjector::create(robot, "cp0", threshold, 50));
  ConfigProjectorPtr_t cp1(ConfigProjector::create(robot, "cp1", threshold, 50));
  ConfigProjectorPtr_t cp2(ConfigProjector::create(robot, "cp2", threshold, 50));
  ConfigProjectorPtr_t cp3(ConfigProjector::create(robot, "cp3", threshold, 50));
  cs0->addConstraint(cp0);
  cs1->addConstraint(cp1);
  cs2->addConstraint(cp2);
  cs3->addConstraint(cp3);
  /* Constraint with non constant right hand side
            / cos t \
        q = |       |
            \ sin t /
  */
  ImplicitPtr_t f0(
      Implicit::create(Identity::create(LiegroupSpace::R2(), "I2"),
          ComparisonTypes_t(2, hpp::constraints::Equality)));
  f0->rightHandSideFunction(Rhs::create(0));
  cp0->add(f0);
  ImplicitPtr_t f1(
      Implicit::create(Identity::create(LiegroupSpace::R2(), "I2"),
          ComparisonTypes_t(2, hpp::constraints::Equality)));
  f1->rightHandSideFunction(Rhs::create(M_PI/2));
  cp1->add(f1);
  ImplicitPtr_t f2(
      Implicit::create(Identity::create(LiegroupSpace::R2(), "I2"),
          ComparisonTypes_t(2, hpp::constraints::Equality)));
  f2->rightHandSideFunction(Rhs::create(M_PI));
  cp2->add(f2);
  ImplicitPtr_t f3(
      Implicit::create(Identity::create(LiegroupSpace::R2(), "I2"),
          ComparisonTypes_t(2, hpp::constraints::Equality)));
  f3->rightHandSideFunction(Rhs::create(3*M_PI/2));
  cp3->add(f3);

  // Create a constrained paths between qi and q{i+1}
  PathPtr_t p0(LocalPath::create(std::make_pair(0, M_PI/2), q0, q1, cs0));
  PathPtr_t p1(LocalPath::create(std::make_pair(0, M_PI/2), q1, q2, cs1));
  PathPtr_t p2(LocalPath::create(std::make_pair(0, M_PI/2), q2, q3, cs2));
  PathPtr_t p3(LocalPath::create(std::make_pair(0, M_PI/2), q3, q0, cs3));
  // Convert them into interpolated paths
  PathPtr_t ip0(InterpolatedPath::create(p0, robot, 4));
  PathPtr_t ip1(InterpolatedPath::create(p1, robot, 4));
  PathPtr_t ip2(InterpolatedPath::create(p2, robot, 4));
  PathPtr_t ip3(InterpolatedPath::create(p3, robot, 4));
  // Concatenate them in a path vector
  PathVectorPtr_t pv(PathVector::create(2, 2));
  pv->appendPath(ip0);
  pv->appendPath(ip1);
  pv->appendPath(ip2);
  pv->appendPath(ip3);
  PathPtr_t ep0(pv->extract(M_PI/4, 5*M_PI/4));
   PathPtr_t ep1(ep0->reverse());
  BOOST_CHECK_CLOSE(ep0->length(), M_PI,
                    std::numeric_limits<float>::epsilon());
  BOOST_CHECK_CLOSE(ep1->length(), M_PI,
                    std::numeric_limits<float>::epsilon());
  for (size_type i=0; i<=50; ++i) {
    bool success;
    value_type s = ((value_type)i)*M_PI/50;
    Configuration_t q = ep0->eval(s, success);
    assert(success);
    BOOST_CHECK(fabs(q[0] - cos(M_PI/4 + s)) <= threshold);
    BOOST_CHECK(fabs(q[1] - sin(M_PI/4 + s)) <= threshold);
    q = ep1->eval(s, success);
    assert(success);
    BOOST_CHECK(fabs(q[0] - cos(5*M_PI/4 - s)) <= threshold);
    BOOST_CHECK(fabs(q[1] - sin(5*M_PI/4 - s)) <= threshold);
  }
}

BOOST_AUTO_TEST_SUITE_END()
