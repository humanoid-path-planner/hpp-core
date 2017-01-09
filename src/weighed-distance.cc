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
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/core/problem.hh>

namespace hpp {
  namespace core {
    namespace {
      struct ComputeWeightStep : public se3::fusion::JointModelVisitor<ComputeWeightStep>
      {
        typedef boost::fusion::vector<const se3::Model &,
                                      const se3::Data &,
                                      const se3::GeometryData &,
                                      value_type & > ArgsType;

        JOINT_MODEL_VISITOR_INIT(ComputeWeightStep);

        template<int N> static value_type largestSingularValue (const Eigen::Matrix<value_type, 3, N>& m)
        {
          typedef Eigen::Matrix<value_type, 3, N> M;
          typedef Eigen::JacobiSVD<M> SVD_t;
          SVD_t svd (m);
          return svd.singularValues()[0];
        }

        template<typename JointModel>
        static void algo(const se3::JointModelBase<JointModel> & jmodel,
            const se3::Model & model,
            const se3::Data & data,
            const se3::GeometryData & geomData,
            value_type & length)
        {
          typedef typename pinocchio::LieGroupTpl::template operation<JointModel>::type LGOp_t;
          typedef Eigen::Matrix<value_type, 3, JointModel::NV>  Block_t;
          typedef Eigen::Matrix<value_type, 3, LGOp_t::NR    > RBlock_t;
          // typedef Eigen::JacobiSVD<RBlock_t> SVD_t;

          if (LGOp_t::NR == 0) {
            length = 1;
            return;
          }

          // SVD_t svd(3, LGOp_t::NR);
          const se3::JointIndex i = jmodel.id();
          JointJacobian_t jacobian(6, data.J.cols());
          Block_t  block;
          RBlock_t rBlock;
          value_type sigma;
          for (se3::JointIndex j = i; j <= (se3::JointIndex)data.lastChild[i]; ++j)
          {
            // Get only three first lines of Jacobian
            se3::getJacobian<true>(model, data, j, jacobian);
            // block = data.oMi[j].rotation() * data.J.block<3, JointModel::NV>(0, jmodel.idx_v());
            block = data.oMi[j].rotation() * jacobian.block<3, JointModel::NV>(0, jmodel.idx_v());

            // se3::getJacobian<true>(model, data, j, jacobian);
            // assert(     jacobian.block(0, jmodel.idx_v(), 3, jmodel.nv())
                // .isApprox(data.J.block(0, jmodel.idx_v(), 3, jmodel.nv()))
                // );

            LGOp_t::getRotationSubJacobian(block, rBlock);
            sigma = largestSingularValue<LGOp_t::NR>(rBlock);
            // svd.compute(rBlock);
            if (length < sigma) {
              length = sigma;
            }
            const value_type & radius = geomData.radius[j];
            // block = data.oMi[j].rotation() * jacobian.block<3, Eigen::Dynamic>(3, rank, 3, ncol);
            block = data.oMi[j].rotation() * data.J.block<3, JointModel::NV>(3, jmodel.idx_v());
            block = data.oMi[j].rotation() * jacobian.block<3, JointModel::NV>(3, jmodel.idx_v());
            LGOp_t::getRotationSubJacobian(block, rBlock);
            sigma = largestSingularValue<LGOp_t::NR>(rBlock);
            // svd.compute(rBlock);
            if (length < radius*sigma) {
              length = radius*sigma;
            }
          }
        }
      };

      template <> void ComputeWeightStep::algo<se3::JointModelComposite>(
          const se3::JointModelBase<se3::JointModelComposite> & jmodel,
          const se3::Model & model,
          const se3::Data & data,
          const se3::GeometryData & geomData,
          value_type & length)
      {
        assert(false);
        // se3::details::Dispatch<SquaredDistanceStep>::run(
            // jmodel.derived(),
            // ArgsType(q0, q1, w, distance));
      }

      template<> value_type ComputeWeightStep::largestSingularValue<0>(const Eigen::Matrix<value_type, 3, 0>&)
      {
        return 0;
      }

      template<> value_type ComputeWeightStep::largestSingularValue<1>(const Eigen::Matrix<value_type, 3, 1>& m)
      {
        return m.norm();
      }

      struct SquaredDistanceStep : public se3::fusion::JointModelVisitor<SquaredDistanceStep>
      {
        typedef boost::fusion::vector<ConfigurationIn_t &,
                ConfigurationIn_t &,
                const value_type &,
                value_type &> ArgsType;

        JOINT_MODEL_VISITOR_INIT(SquaredDistanceStep);

        template<typename JointModel>
        static void algo(const se3::JointModelBase<JointModel> & jmodel,
            ConfigurationIn_t q0,
            ConfigurationIn_t q1,
            const value_type & w,
            value_type & distance)
        {
          distance = ::hpp::pinocchio::LieGroupTpl::template operation<JointModel>::type
            ::squaredDistance(
                jmodel.jointConfigSelector(q0),
                jmodel.jointConfigSelector(q1),
                w);
        }

      };

      template <>
      void SquaredDistanceStep::algo<se3::JointModelComposite>(
          const se3::JointModelBase<se3::JointModelComposite> & jmodel,
          ConfigurationIn_t q0,
          ConfigurationIn_t q1,
          const value_type & w,
          value_type & distance)
      {
        se3::details::Dispatch<SquaredDistanceStep>::run(
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
    (const ProblemPtr_t& problem)
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
      typedef Eigen::Matrix<value_type, 3, Eigen::Dynamic> BlockType;
      typedef Eigen::JacobiSVD<BlockType> SVD_t;

      // Store computation flag
      Device_t::Computation_t flag = robot_->computationFlag ();
      Device_t::Computation_t newflag = static_cast <Device_t::Computation_t>
	(flag | Device_t::JACOBIAN);
      robot_->controlComputation (newflag);
      robot_->computeForwardKinematics ();
      robot_->controlComputation (flag);
      value_type minLength = std::numeric_limits <value_type>::infinity ();

      JointJacobian_t jacobian(6, robot_->numberDof());
      const se3::Model& model = robot_->model();
      const se3::Data& data = robot_->data();
      const se3::GeometryData& geomData = robot_->geomData();
      for (se3::JointIndex i = 1; i < model.joints.size(); ++i)
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

    WeighedDistance::WeighedDistance (const ProblemPtr_t& problem) :
      robot_ (problem->robot()), weights_ ()
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
      value_type res = 0, d;

      const pinocchio::Model& model = robot_->model();
      assert (model.joints.size() <= weights_.size () + 1);
      // Configuration_t qq1 (q1), qq2 (q2);
      // Loop over robot joint
      for( se3::JointIndex i=1; i<(se3::JointIndex) model.njoints; ++i )
      {
        value_type length = weights_ [i-1];
        SquaredDistanceStep::ArgsType args(q1, q2, length * length, d);
        SquaredDistanceStep::run(model.joints[i], args);
        res += d;
      }
      res+=(q1 - q2).tail (robot_->extraConfigSpace ().dimension()).squaredNorm ();
      return sqrt (res);
    }
  } //   namespace core
} // namespace hpp
