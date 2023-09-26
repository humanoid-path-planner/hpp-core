//
// Copyright (c) 2014 CNRS
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

#include <Eigen/SVD>
#include <hpp/core/problem.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/util.hh>
#include <hpp/util/debug.hh>
#include <limits>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace hpp {
namespace core {
namespace {
struct ComputeWeightStep
    : public ::pinocchio::fusion::JointUnaryVisitorBase<ComputeWeightStep> {
  typedef boost::fusion::vector<const pinocchio::Model&, const pinocchio::Data&,
                                const pinocchio::GeomData&, value_type&>
      ArgsType;

  template <int N>
  static value_type largestSingularValue(
      const Eigen::Matrix<value_type, 3, N>& m) {
    typedef Eigen::Matrix<value_type, 3, N> M;
    typedef Eigen::JacobiSVD<M> SVD_t;
    SVD_t svd(m);
    return svd.singularValues()[0];
  }

  template <typename JointModel, int NR>
  static void getRotationSubJacobian(
      const ::pinocchio::JointModelBase<JointModel>& jmodel,
      const pinocchio::Data& data,
      const Eigen::Matrix<value_type, 6, Eigen::Dynamic>& J,
      const pinocchio::JointIndex& j,
      Eigen::Matrix<value_type, 6, NR>& rBlock) {
    typedef
        typename pinocchio::LieGroupTpl::template operation<JointModel>::type
            LGOp_t;
    typedef Eigen::Matrix<value_type, 6, JointModel::NV> Block_t;
    Block_t block;
    // Linear part
    block.template topRows<3>() =
        data.oMi[j].rotation() * J.block<3, JointModel::NV>(0, jmodel.idx_v());
    // Angular part
    block.template bottomRows<3>() =
        data.oMi[j].rotation() * J.block<3, JointModel::NV>(3, jmodel.idx_v());

    LGOp_t::getRotationSubJacobian(block, rBlock);
  }

  template <int NR>
  static value_type computeWeight(
      const Eigen::Matrix<value_type, 6, NR>& rBlock, const value_type& r) {
    value_type linear_sigma =
        largestSingularValue<NR>(rBlock.template topRows<3>());
    value_type angular_sigma =
        largestSingularValue<NR>(rBlock.template bottomRows<3>());

    return std::max(linear_sigma, r * angular_sigma);
  }

  template <typename JointModel>
  static void algo(const ::pinocchio::JointModelBase<JointModel>& jmodel,
                   const pinocchio::Model& model, const pinocchio::Data& data,
                   const pinocchio::GeomData& geomData, value_type& length) {
    typedef
        typename pinocchio::LieGroupTpl::template operation<JointModel>::type
            LGOp_t;
    typedef Eigen::Matrix<value_type, 6, LGOp_t::NR> RBlock_t;

    if (LGOp_t::NR == 0) {
      length = 1;
      return;
    }

    const pinocchio::JointIndex i = jmodel.id();
    JointJacobian_t jacobian(6, data.J.cols());
    RBlock_t rBlock;
    value_type sigma;
    for (pinocchio::JointIndex j = i;
         j <= (pinocchio::JointIndex)data.lastChild[i]; ++j) {
      ::pinocchio::getJointJacobian(model, data, j, ::pinocchio::LOCAL,
                                    jacobian);
      getRotationSubJacobian(jmodel, data, jacobian, j, rBlock);
      const value_type radius =
          geomData.radius[j] +
          (data.oMi[j].translation() - data.oMi[i].translation()).squaredNorm();
      sigma = computeWeight(rBlock, radius);
      if (length < sigma) length = sigma;
    }
  }
};

template <>
value_type ComputeWeightStep::largestSingularValue<0>(
    const Eigen::Matrix<value_type, 3, 0>&) {
  return 0;
}

template <>
value_type ComputeWeightStep::largestSingularValue<1>(
    const Eigen::Matrix<value_type, 3, 1>& m) {
  return m.norm();
}

struct SquaredDistanceStep
    : public ::pinocchio::fusion::JointUnaryVisitorBase<SquaredDistanceStep> {
  typedef boost::fusion::vector<ConfigurationIn_t, ConfigurationIn_t,
                                const value_type&, value_type&>
      ArgsType;

  template <typename JointModel>
  static void algo(const ::pinocchio::JointModelBase<JointModel>& jmodel,
                   ConfigurationIn_t q0, ConfigurationIn_t q1,
                   const value_type& w, value_type& distance) {
    typedef typename ::hpp::pinocchio::LieGroupTpl::template operation<
        JointModel>::type LG_t;
    distance = LG_t().squaredDistance(jmodel.jointConfigSelector(q0),
                                      jmodel.jointConfigSelector(q1), w);
  }
};
}  // namespace

WeighedDistancePtr_t WeighedDistance::create(const DevicePtr_t& robot) {
  WeighedDistance* ptr = new WeighedDistance(robot);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

WeighedDistancePtr_t WeighedDistance::createFromProblem(
    const ProblemConstPtr_t& problem) {
  WeighedDistance* ptr = new WeighedDistance(problem);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

WeighedDistancePtr_t WeighedDistance::createWithWeight(
    const DevicePtr_t& robot, const vector_t& weights) {
  WeighedDistance* ptr = new WeighedDistance(robot, weights);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

WeighedDistancePtr_t WeighedDistance::createCopy(
    const WeighedDistancePtr_t& distance) {
  WeighedDistance* ptr = new WeighedDistance(*distance);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

DistancePtr_t WeighedDistance::clone() const {
  return createCopy(weak_.lock());
}

const vector_t& WeighedDistance::weights() const { return weights_; }

void WeighedDistance::weights(const vector_t& ws) {
  if (ws.size() == weights_.size()) {
    weights_ = ws;
  } else {
    std::ostringstream oss;
    oss << "Distance::weights : size mismatch. Got " << ws.size()
        << ". Expected " << weights_.size() << ".";
    throw std::runtime_error(oss.str());
  }
}

value_type WeighedDistance::getWeight(size_type rank) const {
  return weights_[rank];
}

void WeighedDistance::setWeight(size_type rank, value_type weight) {
  if (rank < weights_.size()) {
    weights_[rank] = weight;
  } else {
    std::ostringstream oss;
    oss << "Distance::setWeight : rank " << rank << " is out of range ("
        << weights_.size() << ").";
    throw std::runtime_error(oss.str());
  }
}

void WeighedDistance::computeWeights() {
  // TODO this test is necessary because of
  // https://github.com/stack-of-tasks/pinocchio/issues/1011
  // It can be removed when the issue is solved.
  if (robot_->configSize() == 0) {
    weights_.resize(0);
    return;
  }
  pinocchio::DeviceSync device(robot_);
  device.computeForwardKinematics(pinocchio::JOINT_POSITION | pinocchio::JACOBIAN);
  value_type minLength = std::numeric_limits<value_type>::infinity();

  const pinocchio::Model& model = device.model();
  const pinocchio::Data& data = device.data();
  const pinocchio::GeomData& geomData = device.geomData();
  weights_.resize(model.joints.size() - 1);
  // TODO when there is only one freeflyer, and the body radius is 0,
  // the weights should be [0,].
  // The algorithm below returns [inf,]
  for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
    value_type length = 0;
    ComputeWeightStep::run(model.joints[i], ComputeWeightStep::ArgsType(
                                                model, data, geomData, length));
    if (minLength > length && length > 0) minLength = length;
    weights_[i - 1] = length;
    for (std::size_t k = 0; k < i; ++k) {
      if (weights_[k] == 0) {
        weights_[k] = minLength;
      }
    }
  }
  hppDout(info, "The weights are " << weights_.transpose());
}

WeighedDistance::WeighedDistance(const DevicePtr_t& robot)
    : robot_(robot), weights_() {
  computeWeights();
}

WeighedDistance::WeighedDistance(const ProblemConstPtr_t& problem)
    : robot_(problem->robot()), weights_() {
  computeWeights();
}

WeighedDistance::WeighedDistance(const DevicePtr_t& robot,
                                 const vector_t& weights)
    : robot_(robot), weights_(weights) {}

WeighedDistance::WeighedDistance(const WeighedDistance& distance)
    : robot_(distance.robot_), weights_(distance.weights_) {}

void WeighedDistance::init(WeighedDistanceWkPtr_t self) { weak_ = self; }

value_type WeighedDistance::impl_distance(ConfigurationIn_t q1,
                                          ConfigurationIn_t q2) const {
  value_type res = 0, d = std::numeric_limits<value_type>::infinity();

  const pinocchio::Model& model = robot_->model();
  assert((size_type)model.joints.size() <= weights_.size() + 1);
  // Configuration_t qq1 (q1), qq2 (q2);
  // Loop over robot joint
  for (pinocchio::JointIndex i = 1; i < (pinocchio::JointIndex)model.njoints;
       ++i) {
    value_type length = weights_[i - 1] * weights_[i - 1];
    SquaredDistanceStep::ArgsType args(q1, q2, length, d);
    SquaredDistanceStep::run(model.joints[i], args);
    res += d;
  }
  res += (q1 - q2).tail(robot_->extraConfigSpace().dimension()).squaredNorm();
  return sqrt(res);
}
}  //   namespace core
}  // namespace hpp
