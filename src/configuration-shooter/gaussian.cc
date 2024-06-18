// Copyright (c) 2018, Joseph Mirabel
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

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <hpp/core/configuration-shooter/gaussian.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace core {
namespace configurationShooter {
namespace liegroup = pinocchio::liegroup;

template <typename LG1, typename LG2>
void computeSigmasAlgo(liegroup::CartesianProductOperation<LG1, LG2> lg,
                       vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower);

template <int N>
void computeSigmasAlgo(liegroup::SpecialOrthogonalOperation<N>,
                       vectorOut_t sigmas, vectorIn_t, vectorIn_t) {
  sigmas.setConstant(2 * M_PI / std::sqrt((value_type)N));
}

template <int N>
void computeSigmasAlgo(liegroup::SpecialEuclideanOperation<N>,
                       vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower) {
  typedef liegroup::CartesianProductOperation<
      liegroup::VectorSpaceOperation<N, true>,
      liegroup::SpecialOrthogonalOperation<N> >
      LG_t;
  computeSigmasAlgo(LG_t(), sigmas, upper, lower);
}

template <int N, bool rot>
void computeSigmasAlgo(liegroup::VectorSpaceOperation<N, rot>,
                       vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower) {
  // TODO isFinite was added after 3.2.4
  // sigmas.array() =
  // (upper.array().isFinite() && lower.array().isFinite())
  // .select (upper - lower, 1);
  for (size_type i = 0; i < sigmas.size(); ++i) {
    if (Eigen::numext::isfinite(upper(i)) && Eigen::numext::isfinite(lower(i)))
      sigmas(i) = upper(i) - lower(i);
    else
      sigmas(i) = 1.;
  }
}

template <typename LG1, typename LG2>
void computeSigmasAlgo(liegroup::CartesianProductOperation<LG1, LG2>,
                       vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower) {
  computeSigmasAlgo(LG1(), sigmas.head(LG1::NV), upper.head(LG1::NQ),
                    lower.head(LG1::NQ));
  computeSigmasAlgo(LG2(), sigmas.tail(LG2::NV), upper.tail(LG2::NQ),
                    lower.tail(LG2::NQ));
}

struct ComputeSigmasStep
    : public ::pinocchio::fusion::JointUnaryVisitorBase<ComputeSigmasStep> {
  typedef boost::fusion::vector<const pinocchio::Model&, vector_t&> ArgsType;

  template <typename JointModel>
  static void algo(const ::pinocchio::JointModelBase<JointModel>& jmodel,
                   const pinocchio::Model& model, vector_t& sigmas) {
    typedef typename pinocchio::DefaultLieGroupMap::operation<JointModel>::type
        LG_t;
    computeSigmasAlgo(LG_t(), jmodel.jointVelocitySelector(sigmas),
                      jmodel.jointConfigSelector(model.upperPositionLimit),
                      jmodel.jointConfigSelector(model.lowerPositionLimit));
  }
};

void Gaussian::impl_shoot(Configuration_t& config) const {
  static boost::random::mt19937 eng;
  vector_t velocity(robot_->numberDof());
  for (size_type i = 0; i < velocity.size(); ++i) {
    boost::random::normal_distribution<value_type> distrib(0, sigmas_[i]);
    velocity[i] = distrib(eng);
  }

  config.resize(robot_->configSize());
  vector_t center(center_);
  if (center.size() == 0) {
    // center has not been initialized, use robot neutral configuration
    center = robot_->neutralConfiguration();
  }
  ::hpp::pinocchio::integrate(robot_, center_, velocity, config);
  ::hpp::pinocchio::saturate(robot_, config);
}

void Gaussian::sigma(const value_type& factor) {
  const pinocchio::Model& model = robot_->model();
  ComputeSigmasStep::ArgsType args(model, sigmas_);
  for (std::size_t i = 1; i < model.joints.size(); ++i)
    ComputeSigmasStep::run(model.joints[i], args);

  sigmas_ *= factor;
}
}  //   namespace configurationShooter
}  //   namespace core
}  // namespace hpp
