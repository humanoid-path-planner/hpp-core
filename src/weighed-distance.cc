//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/core/weighed-distance.hh>

#include <limits>

#include <Eigen/SVD>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/core/problem.hh>

namespace hpp {
  namespace core {
    namespace {
      struct ComputeWeightStep : public ::pinocchio::fusion::JointVisitorBase<ComputeWeightStep>
      {
        typedef boost::fusion::vector<const pinocchio::Model &,
                                      const pinocchio::Data &,
                                      const pinocchio::GeomData &,
                                      value_type & > ArgsType;

        template<int N> static value_type largestSingularValue (const Eigen::Matrix<value_type, 3, N>& m)
        {
          typedef Eigen::Matrix<value_type, 3, N> M;
          typedef Eigen::JacobiSVD<M> SVD_t;
          SVD_t svd (m);
          return svd.singularValues()[0];
        }

        template<typename JointModel, int NR> static void getRotationSubJacobian (
            const ::pinocchio::JointModelBase<JointModel> & jmodel,
            const pinocchio::Data & data,
            const Eigen::Matrix<value_type, 6, Eigen::Dynamic>& J,
            const pinocchio::JointIndex& j,
            Eigen::Matrix<value_type, 6, NR>& rBlock)
        {
          typedef typename pinocchio::LieGroupTpl::template operation<JointModel>::type LGOp_t;
          typedef Eigen::Matrix<value_type, 6, JointModel::NV>  Block_t;
          Block_t block;
          // Linear part
          block.template topRows   <3>() = data.oMi[j].rotation() * J.block<3, JointModel::NV>(0, jmodel.idx_v());
          // Angular part
          block.template bottomRows<3>() = data.oMi[j].rotation() * J.block<3, JointModel::NV>(3, jmodel.idx_v());

          LGOp_t::getRotationSubJacobian(block, rBlock);
        }

        template<int NR> static value_type computeWeight (
            const Eigen::Matrix<value_type, 6, NR>& rBlock,
            const value_type& r)
        {
          value_type linear_sigma  = largestSingularValue<NR>(rBlock.template topRows<3>());
          value_type angular_sigma = largestSingularValue<NR>(rBlock.template bottomRows<3>());

          return std::max(linear_sigma, r * angular_sigma);
        }

        template<typename JointModel>
        static void algo(const ::pinocchio::JointModelBase<JointModel> & jmodel,
            const pinocchio::Model & model,
            const pinocchio::Data & data,
            const pinocchio::GeomData & geomData,
            value_type & length)
        {
          typedef typename pinocchio::LieGroupTpl::template operation<JointModel>::type LGOp_t;
          typedef Eigen::Matrix<value_type, 6, LGOp_t::NR    > RBlock_t;

          if (LGOp_t::NR == 0) {
            length = 1;
            return;
          }

          const pinocchio::JointIndex i = jmodel.id();
          JointJacobian_t jacobian(6, data.J.cols());
          RBlock_t rBlock;
          value_type sigma;
          for (pinocchio::JointIndex j = i; j <= (pinocchio::JointIndex)data.lastChild[i]; ++j)
          {
            ::pinocchio::getJointJacobian(model, data, j, ::pinocchio::LOCAL, jacobian);
            getRotationSubJacobian(jmodel, data, jacobian, j, rBlock);
            const value_type radius = geomData.radius[j]
              + (data.oMi[j].translation() - data.oMi[i].translation()).squaredNorm();
            sigma = computeWeight(rBlock, radius);
            if (length < sigma) length = sigma;
          }
        }
      };

      template <> void ComputeWeightStep::algo< ::pinocchio::JointModelComposite>(
          const ::pinocchio::JointModelBase< ::pinocchio::JointModelComposite> & jmodel,
          const pinocchio::Model & model,
          const pinocchio::Data & data,
          const pinocchio::GeomData & geomData,
          value_type & length)
      {
        hppDout(warning, "The weights for JointModelComposite are not correct."
            " There should be one weight per subjoint.");
        length = 0;
        value_type tmp = 0;
        for (size_t i = 0; i < jmodel.derived().joints.size(); ++i) {
          ComputeWeightStep::run(jmodel.derived().joints[i], ArgsType(model, data, geomData, tmp));
          length += tmp;
        }
      }

      template<> value_type ComputeWeightStep::largestSingularValue<0>(const Eigen::Matrix<value_type, 3, 0>&)
      {
        return 0;
      }

      template<> value_type ComputeWeightStep::largestSingularValue<1>(const Eigen::Matrix<value_type, 3, 1>& m)
      {
        return m.norm();
      }

      struct SquaredDistanceStep : public ::pinocchio::fusion::JointVisitorBase<SquaredDistanceStep>
      {
        typedef boost::fusion::vector<ConfigurationIn_t,
                ConfigurationIn_t,
                const value_type &,
                value_type &> ArgsType;

        template<typename JointModel>
        static void algo(const ::pinocchio::JointModelBase<JointModel> & jmodel,
            ConfigurationIn_t q0,
            ConfigurationIn_t q1,
            const value_type & w,
            value_type & distance)
        {
          typedef typename ::hpp::pinocchio::LieGroupTpl::template operation<JointModel>::type LG_t;
          distance = LG_t().squaredDistance(
                jmodel.jointConfigSelector(q0),
                jmodel.jointConfigSelector(q1),
                w);
        }

      };

      template <>
      void SquaredDistanceStep::algo< ::pinocchio::JointModelComposite>(
          const ::pinocchio::JointModelBase< ::pinocchio::JointModelComposite> & jmodel,
          ConfigurationIn_t q0,
          ConfigurationIn_t q1,
          const value_type & w,
          value_type & distance)
      {
        ::pinocchio::details::Dispatch<SquaredDistanceStep>::run(
            jmodel.derived(),
            ArgsType(q0, q1, w, distance));
      }
    }

    std::ostream& operator<< (std::ostream& os, const std::vector <value_type>& v)
    {
      for (std::size_t i=0; i<v.size (); ++i) {
	os << v [i] << ",";
      }
      return os;
    }

    WeighedDistancePtr_t WeighedDistance::create (const DevicePtr_t& robot)
    {
      WeighedDistance* ptr = new WeighedDistance (robot);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t WeighedDistance::createFromProblem
    (const Problem& problem)
    {
      WeighedDistance* ptr = new WeighedDistance (problem);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t
    WeighedDistance::createWithWeight (const DevicePtr_t& robot,
			     const std::vector <value_type>& weights)
    {
      WeighedDistance* ptr = new WeighedDistance (robot, weights);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t WeighedDistance::createCopy
	(const WeighedDistancePtr_t& distance)
    {
      WeighedDistance* ptr = new WeighedDistance (*distance);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    DistancePtr_t WeighedDistance::clone () const
    {
      return createCopy (weak_.lock ());
    }

    value_type WeighedDistance::getWeight( std::size_t rank ) const
    {
      return weights_[rank];
    }

    void WeighedDistance::setWeight (std::size_t rank, value_type weight )
    {
      if ( rank < weights_.size() )
      {
	weights_[rank] = weight;
      }
      else {
	std::ostringstream oss;
	oss << "Distance::setWeight : rank " << rank << " is out of range ("
	    << weights_.size () << ").";
	throw std::runtime_error(oss.str ());
      }
    }

    void WeighedDistance::computeWeights ()
    {
      // Store computation flag
      pinocchio::Computation_t flag = robot_->computationFlag ();
      pinocchio::Computation_t newflag = static_cast <pinocchio::Computation_t>
	(flag | pinocchio::JACOBIAN);
      robot_->controlComputation (newflag);
      robot_->computeForwardKinematics ();
      robot_->controlComputation (flag);
      value_type minLength = std::numeric_limits <value_type>::infinity ();

      JointJacobian_t jacobian(6, robot_->numberDof());
      const pinocchio::Model& model = robot_->model();
      const pinocchio::Data& data = robot_->data();
      const pinocchio::GeomData& geomData = robot_->geomData();
      for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i)
      {
	  value_type length = 0;
          ComputeWeightStep::run(model.joints[i],
              ComputeWeightStep::ArgsType(model, data, geomData, length));
	  if (minLength > length && length > 0) minLength = length;
	  weights_.push_back (length);
	for (std::size_t k=0; k < weights_.size (); ++k) {
	  if (weights_ [k] == 0) {
	    weights_ [k] = minLength;
	  }
	}
      }
      hppDout(info, "The weights are " << Eigen::Map<vector_t>(weights_.data(), weights_.size()).transpose());
    }

    WeighedDistance::WeighedDistance (const DevicePtr_t& robot) :
      robot_ (robot), weights_ ()
    {
      computeWeights ();
    }

    WeighedDistance::WeighedDistance (const Problem& problem) :
      robot_ (problem.robot()), weights_ ()
    {
      computeWeights ();
    }

    WeighedDistance::WeighedDistance (const DevicePtr_t& robot,
				      const std::vector <value_type>& weights) :
      robot_ (robot), weights_ (weights)
    {
    }

    WeighedDistance::WeighedDistance (const WeighedDistance& distance) :
      robot_ (distance.robot_),
      weights_ (distance.weights_)
    {
    }

    void WeighedDistance::init (WeighedDistanceWkPtr_t self)
    {
      weak_ = self;
    }

    value_type WeighedDistance::impl_distance (ConfigurationIn_t q1,
					       ConfigurationIn_t q2) const
    {
      value_type res = 0, d = std::numeric_limits <value_type>::infinity ();

      const pinocchio::Model& model = robot_->model();
      assert (model.joints.size() <= weights_.size () + 1);
      // Configuration_t qq1 (q1), qq2 (q2);
      // Loop over robot joint
      for( pinocchio::JointIndex i=1; i<(pinocchio::JointIndex) model.njoints; ++i )
      {
        value_type length = weights_ [i-1] * weights_ [i-1];
        SquaredDistanceStep::ArgsType args(q1, q2, length, d);
        SquaredDistanceStep::run(model.joints[i], args);
        res += d;
      }
      res+=(q1 - q2).tail (robot_->extraConfigSpace ().dimension()).squaredNorm ();
      return sqrt (res);
    }
  } //   namespace core
} // namespace hpp
