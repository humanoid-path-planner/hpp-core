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

#include <coal/math/transform.h>

#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/time-parameterization/polynomial.hh>
#include <pinocchio/fwd.hpp>

BOOST_AUTO_TEST_SUITE(test_hpp_core)

using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::ConfigurationOut_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointConstPtr_t;
using hpp::pinocchio::JointPtr_t;

using namespace hpp::core;

HPP_PREDEF_CLASS(LocalPath);
typedef std::shared_ptr<LocalPath> LocalPathPtr_t;

class LocalPath : public Path {
 public:
  virtual ~LocalPath() throw() {}
  virtual bool impl_compute(ConfigurationOut_t configuration,
                            value_type t) const {
    value_type alpha(t - timeRange().first /
                             (timeRange().second - timeRange().first));
    configuration = (1 - alpha) * q1_ + alpha * q2_;
    return true;
  }

  static LocalPathPtr_t create(const interval_t& interval) {
    LocalPath* ptr(new LocalPath(interval));
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

BOOST_AUTO_TEST_SUITE_END()
