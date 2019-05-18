// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/configuration-shooter/gaussian.hh>

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>
# include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {
      namespace liegroup = pinocchio::liegroup;

      template <typename LG1, typename LG2>
      void computeSigmasAlgo (liegroup::CartesianProductOperation<LG1, LG2> lg,
            vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower);

      template <int N>
      void computeSigmasAlgo (liegroup::SpecialOrthogonalOperation<N>,
            vectorOut_t sigmas, vectorIn_t, vectorIn_t)
      {
        sigmas.setConstant (2*M_PI/std::sqrt((value_type)N));
      }

      template <int N>
      void computeSigmasAlgo (liegroup::SpecialEuclideanOperation<N>,
            vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower)
      {
        typedef liegroup::CartesianProductOperation<
          liegroup::VectorSpaceOperation<N,true>,
          liegroup::SpecialOrthogonalOperation<N>
            > LG_t;
        computeSigmasAlgo (LG_t(), sigmas, upper, lower);
      }

      template <int N, bool rot>
      void computeSigmasAlgo (liegroup::VectorSpaceOperation<N, rot>,
            vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower)
      {
        // TODO isFinite was added after 3.2.4
        // sigmas.array() =
          // (upper.array().isFinite() && lower.array().isFinite())
          // .select (upper - lower, 1);
        for (size_type i = 0; i < sigmas.size(); ++i) {
          if (Eigen::numext::isfinite (upper(i))
              && Eigen::numext::isfinite (lower(i)))
            sigmas(i) = upper(i)-lower(i);
          else
            sigmas(i) = 1.;
        }
      }

      template <typename LG1, typename LG2>
      void computeSigmasAlgo (liegroup::CartesianProductOperation<LG1, LG2>,
            vectorOut_t sigmas, vectorIn_t upper, vectorIn_t lower)
      {
        computeSigmasAlgo (LG1(), sigmas.head(LG1::NV), upper.head(LG1::NQ), lower.head(LG1::NQ));
        computeSigmasAlgo (LG2(), sigmas.tail(LG2::NV), upper.tail(LG2::NQ), lower.tail(LG2::NQ));
      }

      struct ComputeSigmasStep : public ::pinocchio::fusion::JointVisitorBase<ComputeSigmasStep>
      {
        typedef boost::fusion::vector<const pinocchio::Model&, vector_t&> ArgsType;

        template<typename JointModel>
          static void algo(const ::pinocchio::JointModelBase<JointModel> & jmodel,
              const pinocchio::Model& model,
              vector_t& sigmas)
          {
            typedef typename pinocchio::DefaultLieGroupMap::operation<JointModel>::type LG_t;
            computeSigmasAlgo (LG_t(), 
                jmodel.jointVelocitySelector (sigmas),
                jmodel.jointConfigSelector   (model.upperPositionLimit),
                jmodel.jointConfigSelector   (model.lowerPositionLimit));
          }
      };

      template<>
      void ComputeSigmasStep::algo< ::pinocchio::JointModelComposite>(const ::pinocchio::JointModelBase< ::pinocchio::JointModelComposite> & jmodel,
              const pinocchio::Model& model,
              vector_t& sigmas)
      {
        ::pinocchio::details::Dispatch<ComputeSigmasStep>::run(jmodel.derived(), ComputeSigmasStep::ArgsType(model, sigmas));
      }

      void Gaussian::shoot (Configuration_t& config) const
      {
        static boost::random::mt19937 eng;
        vector_t velocity (robot_->numberDof());
        for (size_type i = 0; i < velocity.size(); ++i)
        {
          boost::random::normal_distribution<value_type> distrib(0, sigmas_[i]);
          velocity[i] = distrib (eng);
        }

        config.resize(robot_->configSize ());
        ::hpp::pinocchio::integrate (robot_, center_, velocity, config);
        ::hpp::pinocchio::saturate  (robot_, config);
      }

      void Gaussian::sigma(const value_type& factor)
      {
        const pinocchio::Model& model = robot_->model();
        ComputeSigmasStep::ArgsType args (model, sigmas_);
        for(std::size_t i = 1; i < model.joints.size(); ++i)
          ComputeSigmasStep::run (model.joints[i], args);

        sigmas_ *= factor;
      }
    } //   namespace configurationShooter
  } //   namespace core
} // namespace hpp
