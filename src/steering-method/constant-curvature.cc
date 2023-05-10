//
// Copyright (c) 2017 CNRS
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

#include <boost/serialization/vector.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/core/steering-method/constant-curvature.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/serialization.hh>
#include <pinocchio/serialization/eigen.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace core {
namespace steeringMethod {
namespace details {
using std::max;

/// \param t0, t1 angles between \f$ 0 \f$ and \f$ 2 \pi \f$.
value_type absCosineMax(const value_type& t0, const value_type& t1) {
  assert(0 <= t0 && t0 <= t1 && t1 <= 2 * M_PI);
  if (t1 <= M_PI) return max(fabs(cos(t0)), fabs(cos(t1)));
  // M_PI < t1 <= 2*M_PI
  if (t0 <= M_PI) return 1.;
  // M_PI < t0 <= t1 <= 2*M_PI
  return max(fabs(cos(t0)), fabs(cos(t1)));
}

/// \param t0, t1 angles between \f$ 0 \f$ and \f$ 2 \pi \f$.
value_type absSineMax(const value_type& t0, const value_type& t1) {
  assert(0 <= t0 && t0 <= t1 && t1 <= 2 * M_PI);
  if (t1 <= M_PI_2) return sin(t1);
  if (t0 <= M_PI_2) return 1.;
  if (t0 >= 3 * M_PI_2) return -sin(t0);
  // M_PI_2 < t0 < 3*M_PI_2
  if (t1 >= 3 * M_PI_2) return 1.;
  return max(sin(t0), sin(t1));
}
}  // namespace details

ConstantCurvaturePtr_t ConstantCurvature::create(
    const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
    value_type curveLength, value_type pathLength, value_type curvature,
    size_type xyId, size_type rzId, const JointPtr_t rz,
    const std::vector<JointPtr_t> wheels,
    const ConstraintSetPtr_t& constraints) {
  ConstantCurvature* ptr;
  if (constraints)
    ptr = new ConstantCurvature(robot, init, end, curveLength, pathLength,
                                curvature, xyId, rzId, rz, wheels, constraints);
  else
    ptr = new ConstantCurvature(robot, init, end, curveLength, pathLength,
                                curvature, xyId, rzId, rz, wheels);
  ConstantCurvaturePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConstantCurvaturePtr_t ConstantCurvature::createCopy(
    const ConstantCurvaturePtr_t& other) {
  ConstantCurvature* ptr(new ConstantCurvature(*other));
  ConstantCurvaturePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConstantCurvaturePtr_t ConstantCurvature::createCopy(
    const ConstantCurvaturePtr_t& other,
    const ConstraintSetPtr_t& constraints) {
  ConstantCurvature* ptr = new ConstantCurvature(*other, constraints);
  ConstantCurvaturePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

/// Return a shared pointer to a copy of this
PathPtr_t ConstantCurvature::copy() const { return createCopy(weak_.lock()); }

std::ostream& ConstantCurvature::print(std::ostream& os) const {
  os << "-- ConstantCurvature" << std::endl;
  os << "from " << initial_.transpose() << std::endl;
  os << "  length: " << forward_ * paramLength() << std::endl;
  os << "  curvature: " << curvature_ << std::endl;
  return os;
}

ConstantCurvature::ConstantCurvature(
    const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
    value_type curveLength, value_type pathLength, value_type curvature,
    size_type xyId, size_type rzId, const JointPtr_t rz,
    const std::vector<JointPtr_t> wheels, ConstraintSetPtr_t constraints)
    : Path(std::make_pair(0., fabs(pathLength)), robot->configSize(),
           robot->numberDof(), constraints),
      robot_(robot),
      initial_(init),
      end_(end),
      curveLength_(curveLength),
      curvature_(curvature),
      xyId_(xyId),
      rzId_(rzId),
      forward_(curveLength > 0 ? 1 : -1) {
  // Find rank of translation and rotation in velocity vectors
  // Hypothesis: degrees of freedom all belong to a planar joint or
  // xyId_ belong to a tranlation joint, rzId_ belongs to a SO2 joint.
  JointPtr_t joint(robot_->getJointAtConfigRank(xyId_));
  size_type offset(xyId_ - joint->rankInConfiguration());
  dxyId_ = joint->rankInVelocity() + offset;
  joint = robot_->getJointAtConfigRank(rzId_);
  offset = rzId_ - joint->rankInConfiguration();
  drzId_ = joint->rankInVelocity() + offset;
  setWheelJoints(rz, wheels);
  impl_compute(end_, paramRange().second);
}

ConstantCurvature::ConstantCurvature(
    const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
    value_type curveLength, value_type pathLength, value_type curvature,
    size_type xyId, size_type rzId, const JointPtr_t rz,
    const std::vector<JointPtr_t> wheels)
    : Path(std::make_pair(0., fabs(pathLength)), robot->configSize(),
           robot->numberDof()),
      robot_(robot),
      initial_(init),
      end_(end),
      curveLength_(curveLength),
      curvature_(curvature),
      xyId_(xyId),
      rzId_(rzId),
      forward_(curveLength > 0 ? 1 : -1) {
  // Find rank of translation and rotation in velocity vectors
  // Hypothesis: degrees of freedom all belong to a planar joint or
  // xyId_ belong to a tranlation joint, rzId_ belongs to a SO2 joint.
  JointPtr_t joint(robot_->getJointAtConfigRank(xyId_));
  size_type offset(xyId_ - joint->rankInConfiguration());
  dxyId_ = joint->rankInVelocity() + offset;
  joint = robot_->getJointAtConfigRank(rzId_);
  offset = rzId_ - joint->rankInConfiguration();
  drzId_ = joint->rankInVelocity() + offset;
  setWheelJoints(rz, wheels);
  impl_compute(end_, paramRange().second);
}

ConstantCurvature::ConstantCurvature(const ConstantCurvature& other)
    : Path(other),
      robot_(other.robot_),
      initial_(other.initial_),
      end_(other.end_),
      curveLength_(other.curveLength_),
      curvature_(other.curvature_),
      xyId_(other.xyId_),
      rzId_(other.rzId_),
      dxyId_(other.dxyId_),
      drzId_(other.drzId_),
      forward_(other.forward_),
      wheels_(other.wheels_) {}

ConstantCurvature::ConstantCurvature(const ConstantCurvature& other,
                                     const ConstraintSetPtr_t& constraints)
    : parent_t(other, constraints),
      robot_(other.robot_),
      initial_(other.initial_),
      end_(other.end_),
      curveLength_(other.curveLength_),
      curvature_(other.curvature_),
      xyId_(other.xyId_),
      rzId_(other.rzId_),
      dxyId_(other.dxyId_),
      drzId_(other.drzId_),
      forward_(other.forward_),
      wheels_(other.wheels_) {}

bool ConstantCurvature::impl_compute(ConfigurationOut_t result,
                                     value_type param) const {
  const value_type L = paramLength();
  // Does a linear interpolation on all the joints.
  const value_type u = (L == 0) ? 0 : ((param - paramRange().first) / L);
  pinocchio::interpolate<pinocchio::RnxSOnLieGroupMap>(robot_, initial_, end_,
                                                       u, result);

  value_type t(u * curveLength_);
  value_type x0(initial_[xyId_ + 0]), y0(initial_[xyId_ + 1]);
  value_type c0(initial_[rzId_ + 0]), s0(initial_[rzId_ + 1]);
  value_type x, y;
  value_type c(cos(curvature_ * t)), s(sin(curvature_ * t));

  if (curvature_ == 0) {
    x = x0 + t * c0;
    y = y0 + t * s0;
  } else {
    value_type r(1. / curvature_);
    x = x0 + r * (s0 * (c - 1) + c0 * s);
    y = y0 + r * (c0 * (1 - c) + s0 * s);
  }
  result[xyId_ + 0] = x;
  result[xyId_ + 1] = y;
  result[rzId_ + 0] = c0 * c - s0 * s;
  result[rzId_ + 1] = c0 * s + s0 * c;

  // Set wheel joint positions
  for (std::vector<Wheels_t>::const_iterator w = wheels_.begin();
       w < wheels_.end(); ++w) {
    result[w->j->rankInConfiguration()] = w->value;
  }
  return true;
}

void ConstantCurvature::impl_derivative(vectorOut_t result,
                                        const value_type& param,
                                        size_type order) const {
  const value_type L = paramLength();
  // Does a linear interpolation on all the joints.
  const value_type u = (L == 0) ? 0 : ((param - paramRange().first) / L);
  if (order == 1) {
    pinocchio::difference<pinocchio::RnxSOnLieGroupMap>(robot_, end_, initial_,
                                                        result);
    result /= L;
  } else if (order > 1) {
    result.setZero();
  }

  value_type t(u * curveLength_);
  value_type beta(curveLength_ / L);
  if (L == 0) beta = 1;
  value_type alpha(fabs(beta));
  if (forward_ == -1 && order % 2 == 1) alpha *= -1;
  value_type c0(initial_[rzId_ + 0]), s0(initial_[rzId_ + 1]);
  value_type dx, dy, dtheta = 0;
  value_type c(cos(curvature_ * t)), s(sin(curvature_ * t));

  if (order <= 0) {
    std::ostringstream oss;
    oss << "order of derivative (" << order << ") should be positive.";
    throw std::runtime_error(oss.str().c_str());
  }
  if (order == 1) {
    dx = alpha * (c0 * c - s0 * s);
    dy = alpha * (c0 * s + s0 * c);
    dtheta = alpha * curvature_;
  } else {
    switch (order % 4) {
      case 2:
        dx = alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * s + s0 * c);
        dy = -alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * c - s0 * s);
        break;
      case 3:
        dx = -alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * c - s0 * s);
        dy = -alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * s + s0 * c);
        break;
      case 0:
        dx = alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * s + s0 * c);
        dy = -alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * c - s0 * s);
        break;
      case 1:
        dx = alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * c - s0 * s);
        dy = alpha * pow(curvature_ * beta, (value_type)(order - 1)) *
             (c0 * s + s0 * c);
        break;
    }
  }

  result[dxyId_ + 0] = dx;
  result[dxyId_ + 1] = dy;
  result[drzId_] = dtheta;
  // Express velocity in local frame
  Eigen::Matrix<value_type, 2, 2> R;
  R.col(0) << c0 * c - s0 * s, c0 * s + s0 * c;
  R.col(1) << -R(1, 0), R(0, 0);
  result.segment<2>(dxyId_) = R.transpose() * result.segment<2>(dxyId_);

  // Set wheel joint velocities
  for (std::vector<Wheels_t>::const_iterator w = wheels_.begin();
       w < wheels_.end(); ++w) {
    result[w->j->rankInVelocity()] = 0;
  }
}

void ConstantCurvature::impl_velocityBound(vectorOut_t bound,
                                           const value_type& param0,
                                           const value_type& param1) const {
  // Does a linear interpolation on all the joints.
  if (paramRange().first == paramRange().second) {
    bound.setZero();
    return;
  }
  const value_type L = paramLength();
  assert(L > 0);
  pinocchio::difference<pinocchio::RnxSOnLieGroupMap>(robot_, end_, initial_,
                                                      bound);
  bound.noalias() = bound.cwiseAbs() / L;

  value_type alpha(fabs(curveLength_) / L);

  // Compute max of
  // dx = alpha * (c0 * c - s0 * s);
  // dy = alpha * (c0 * s + s0 * c);
  // dtheta = alpha * curvature_;
  // onto [ param0, param1 ]
  value_type Mx = std::numeric_limits<value_type>::quiet_NaN(),
             My = std::numeric_limits<value_type>::quiet_NaN();

  if (curvature_ != 0) {
    value_type t0 = curvature_ * curveLength_ * (param0 - paramRange().first) /
                    L,
               t1 = curvature_ * curveLength_ * (param1 - paramRange().first) /
                    L;
    if (t0 > t1) std::swap(t0, t1);
    if (t1 - t0 >= M_PI)
      Mx = My = 1;
    else {
      // Compute thetaI
      const value_type c0(initial_[rzId_ + 0]), s0(initial_[rzId_ + 1]);
      const value_type tI(atan2(s0, c0));
      t0 += tI;
      t1 += tI;
      // max(|c0 * c - s0 * s|) = max(|cos(t)|) for t in [t0, t1]
      // max(|c0 * s + s0 * c|) = max(|sin(t)|) for t in [t0, t1]

      value_type kd = std::floor(t0 * M_1_PI / 2) * 2 * M_PI;
      t0 -= kd;
      t1 -= kd;
      assert(0 <= t0 && t0 <= t1 && t1 <= 2 * M_PI);

      Mx = details::absCosineMax(t0, t1);
      My = details::absSineMax(t0, t1);
    }
  } else {
    Mx = 1.;
    My = 0.;
  }
  assert(Mx >= 0. && My >= 0.);
  assert(Mx <= 1. && My <= 1.);

  bound[dxyId_ + 0] = alpha * Mx;
  bound[dxyId_ + 1] = alpha * My;
  bound[drzId_] = alpha * std::fabs(curvature_);

  // Set wheel joint velocities
  for (std::vector<Wheels_t>::const_iterator w = wheels_.begin();
       w < wheels_.end(); ++w) {
    bound[w->j->rankInVelocity()] = 0;
  }
}

PathPtr_t ConstantCurvature::impl_extract(
    const interval_t& paramInterval) const {
  assert(!timeParameterization());
  assert(timeRange().second - timeRange().first >= 0);
  value_type L(timeRange().second - timeRange().first);
  value_type tmin(paramInterval.first), tmax(paramInterval.second);
  value_type curveLength = 0, pathLength = 0;
  if (L != 0) {
    curveLength = ((tmax - tmin) / L * curveLength_);
    pathLength = fabs(tmax - tmin) / L * length();
  }
  Configuration_t init(robot_->configSize()), end(robot_->configSize());
  auto tr = timeRange();
  bool res(true);
  if (tmin == tr.first)
    init = initial_;
  else if (tmin == tr.second)
    init = end_;
  else
    res = impl_compute(init, tmin);
  assert(res);
  (void)res;
  if (tmax == tr.first)
    end = initial_;
  else if (tmax == tr.second)
    end = end_;
  else
    res = impl_compute(end, tmax);
  assert(res);
  ConstantCurvaturePtr_t result(createCopy(weak_.lock()));
  // swap to avoid memory allocation.
  result->initial_.swap(init);
  result->end_.swap(end);
  result->curveLength_ = curveLength;
  result->timeRange(interval_t(0, pathLength));
  result->forward_ = curveLength > 0 ? 1 : -1;
  return result;
}

PathPtr_t ConstantCurvature::reverse() const {
  assert(!timeParameterization());
  assert(timeRange().second - timeRange().first >= 0);

  ConstantCurvaturePtr_t result(createCopy(weak_.lock()));
  result->initial_ = end_;
  result->end_ = initial_;
  result->curveLength_ = -curveLength_;
  result->timeRange(interval_t(0, length()));
  result->forward_ = curveLength_ < 0 ? 1 : -1;
  return result;
}

inline value_type meanBounds(const JointPtr_t& j, const size_type& i) {
  return (j->upperBound(i) + j->lowerBound(i)) / 2;
}

inline value_type saturate(const value_type& v, const JointPtr_t& j,
                           const size_type& i) {
  return std::min(j->upperBound(i), std::max(j->lowerBound(i), v));
}

void ConstantCurvature::setWheelJoints(const JointPtr_t rz,
                                       const std::vector<JointPtr_t> wheels) {
  Transform3f zt(rz->currentTransformation().inverse());
  wheels_.resize(wheels.size());
  std::size_t rk = 0;
  if (curvature_ == 0) {
    for (std::vector<JointPtr_t>::const_iterator _wheels = wheels.begin();
         _wheels != wheels.end(); ++_wheels) {
      wheels_[rk].j = *_wheels;
      wheels_[rk].value = meanBounds(wheels_[rk].j, 0);
      ++rk;
    }
  } else {
    value_type rho(1. / curvature_);
    for (std::vector<JointPtr_t>::const_iterator _wheels = wheels.begin();
         _wheels != wheels.end(); ++_wheels) {
      wheels_[rk].j = *_wheels;
      wheels_[rk].value = meanBounds(wheels_[rk].j, 0);
      const vector3_t wheelPos =
          zt.act(wheels_[rk].j->currentTransformation().translation());
      const value_type value(std::atan(wheelPos[0] / (rho - wheelPos[1])));
      wheels_[rk].value =
          saturate(meanBounds(wheels_[rk].j, 0) + value, *_wheels, 0);
      ++rk;
    }
  }
}

template <class Archive>
void ConstantCurvature::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<Path>(*this));
  ar& BOOST_SERIALIZATION_NVP(robot_);
  serialization::remove_duplicate::serialize_vector(ar, "initial", initial_,
                                                    version);
  serialization::remove_duplicate::serialize_vector(ar, "end", end_, version);
  ar& BOOST_SERIALIZATION_NVP(curveLength_);
  ar& make_nvp("curvature_", const_cast<value_type&>(curvature_));
  ar& make_nvp("xyId_", const_cast<size_type&>(xyId_));
  ar& make_nvp("rzId_", const_cast<size_type&>(rzId_));
  ar& BOOST_SERIALIZATION_NVP(dxyId_);
  ar& BOOST_SERIALIZATION_NVP(drzId_);
  ar& BOOST_SERIALIZATION_NVP(forward_);
  ar& BOOST_SERIALIZATION_NVP(wheels_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ConstantCurvature);
}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp

namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive& ar,
               hpp::core::steeringMethod::ConstantCurvature::Wheels_t& c,
               const unsigned int version) {
  (void)version;
  ar& make_nvp("value", c.value);
  ar& make_nvp("joint", c.j);
}
}  // namespace serialization
}  // namespace boost

BOOST_CLASS_EXPORT(hpp::core::steeringMethod::ConstantCurvature)
