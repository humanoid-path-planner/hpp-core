//
// Copyright (c) 2017 CNRS
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

#include <hpp/core/steering-method/constant-curvature.hh>

#include <pinocchio/spatial/se3.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {

      ConstantCurvaturePtr_t ConstantCurvature::create
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type curveLength, value_type pathLength, value_type curvature,
       size_type xyId, size_type rzId,
       const JointPtr_t rz, const std::vector<JointPtr_t> wheels,
       const ConstraintSetPtr_t& constraints)
      {
        ConstantCurvature* ptr;
        if (constraints)
          ptr = new ConstantCurvature (
              robot, init, end, curveLength, pathLength,
              curvature, xyId, rzId, rz, wheels, constraints);
        else
          ptr = new ConstantCurvature (
              robot, init, end, curveLength, pathLength,
              curvature, xyId, rzId, rz, wheels);
        ConstantCurvaturePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      ConstantCurvaturePtr_t ConstantCurvature::createCopy
      (const ConstantCurvaturePtr_t& other)
      {
        ConstantCurvature* ptr (new ConstantCurvature (*other));
        ConstantCurvaturePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      ConstantCurvaturePtr_t ConstantCurvature::createCopy
      (const ConstantCurvaturePtr_t& other,
       const ConstraintSetPtr_t& constraints)
      {
	ConstantCurvature* ptr = new ConstantCurvature (*other, constraints);
	ConstantCurvaturePtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Return a shared pointer to a copy of this
      PathPtr_t ConstantCurvature::copy () const
      {
	return createCopy (weak_.lock ());
      }

      std::ostream& ConstantCurvature::print (std::ostream &os) const
      {
        os << "-- ConstantCurvature" << std::endl;
        os << "from " << initial_.transpose () << std::endl;
        os << "  length: " << forward_ * paramLength() << std::endl;
        os << "  curvature: " << curvature_ << std::endl;
        return os;
      }

      ConstantCurvature::ConstantCurvature
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type curveLength, value_type pathLength, value_type curvature,
       size_type xyId, size_type rzId,
       const JointPtr_t rz, const std::vector<JointPtr_t> wheels,
       ConstraintSetPtr_t constraints) :
        Path (std::make_pair (0., fabs (pathLength)), robot->configSize (),
              robot->numberDof (), constraints), robot_ (robot),
        initial_ (init), end_ (end),
        curveLength_ (curveLength), curvature_ (curvature),
        xyId_ (xyId), rzId_ (rzId), forward_ (curveLength > 0 ? 1 : -1)
      {
        // Find rank of translation and rotation in velocity vectors
        // Hypothesis: degrees of freedom all belong to a planar joint or
        // xyId_ belong to a tranlation joint, rzId_ belongs to a SO2 joint.
        JointPtr_t joint (robot_->getJointAtConfigRank (xyId_));
        size_type offset (xyId_ - joint->rankInConfiguration ());
        dxyId_ = joint->rankInVelocity () + offset;
        joint = robot_->getJointAtConfigRank (rzId_);
        offset = rzId_ - joint->rankInConfiguration ();
        drzId_ = joint->rankInVelocity () + offset;
        setWheelJoints (rz, wheels);
        impl_compute (end_, paramRange ().second);
      }

      ConstantCurvature::ConstantCurvature
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type curveLength, value_type pathLength, value_type curvature,
       size_type xyId, size_type rzId,
       const JointPtr_t rz, const std::vector<JointPtr_t> wheels) :
        Path (std::make_pair (0., fabs (pathLength)), robot->configSize (),
              robot->numberDof ()), robot_ (robot),
        initial_ (init), end_ (end), curveLength_ (curveLength),
        curvature_ (curvature), xyId_ (xyId),
        rzId_ (rzId), forward_ (curveLength > 0 ? 1 : -1)
      {
        // Find rank of translation and rotation in velocity vectors
        // Hypothesis: degrees of freedom all belong to a planar joint or
        // xyId_ belong to a tranlation joint, rzId_ belongs to a SO2 joint.
        JointPtr_t joint (robot_->getJointAtConfigRank (xyId_));
        size_type offset (xyId_ - joint->rankInConfiguration ());
        dxyId_ = joint->rankInVelocity () + offset;
        joint = robot_->getJointAtConfigRank (rzId_);
        offset = rzId_ - joint->rankInConfiguration ();
        drzId_ = joint->rankInVelocity () + offset;
        setWheelJoints (rz, wheels);
        impl_compute (end_, paramRange ().second);
      }

      ConstantCurvature::ConstantCurvature (const ConstantCurvature& other) :
        Path (other), robot_ (other.robot_), initial_ (other.initial_),
        end_ (other.end_), curveLength_ (other.curveLength_),
        curvature_ (other.curvature_),  xyId_ (other.xyId_),
        rzId_ (other.rzId_), forward_ (other.forward_), wheels_ (other.wheels_)
      {
      }

      ConstantCurvature::ConstantCurvature
      (const ConstantCurvature& other, const ConstraintSetPtr_t& constraints) :
        parent_t (other, constraints), robot_ (other.robot_),
        initial_ (other.initial_), end_ (other.end_),
        curveLength_ (other.curveLength_), curvature_ (other.curvature_),
        xyId_ (other.xyId_), rzId_ (other.rzId_), forward_ (other.forward_),
        wheels_ (other.wheels_)
      {
      }

      bool ConstantCurvature::impl_compute (ConfigurationOut_t result,
                                            value_type param) const
      {
        const value_type L = paramLength();
        // Does a linear interpolation on all the joints.
        const value_type u = (L == 0) ? 0 : ((param - paramRange ().first) / L );
        pinocchio::interpolate <pinocchio::RnxSOnLieGroupMap>(robot_, initial_, end_, u, result);

        value_type t (u * curveLength_);
        value_type x0 (initial_ [xyId_ + 0]), y0 (initial_ [xyId_ + 1]);
        value_type c0 (initial_ [rzId_ + 0]), s0 (initial_ [rzId_ + 1]);
        value_type x, y;
        value_type c (cos (curvature_ * t)), s (sin (curvature_ * t));

        if (curvature_ == 0) {
          x  = x0 + t * c0;
          y  = y0 + t * s0;
        } else {
          value_type r (1./curvature_);
          x = x0 + r * (s0 * (c - 1) + c0 * s);
          y = y0 + r * (c0 * (1 - c) + s0 * s);
        }
        result [xyId_ + 0] = x;
        result [xyId_ + 1] = y;
        result [rzId_ + 0] = c0 * c - s0 * s;
        result [rzId_ + 1] = c0 * s + s0 * c;

        // Set wheel joint positions
        for (std::vector<Wheels_t>::const_iterator w = wheels_.begin ();
             w < wheels_.end (); ++w) {
          result [w->j->rankInConfiguration ()] = w->value;
        }
        return true;
      }

      void ConstantCurvature::impl_derivative
      (vectorOut_t result, const value_type& param, size_type order) const
      {
        const value_type L = paramLength();
        // Does a linear interpolation on all the joints.
        const value_type u = (L == 0) ? 0 : ((param - paramRange ().first) / L );
        if (order == 1) {
          pinocchio::difference <pinocchio::RnxSOnLieGroupMap>
            (robot_, end_, initial_, result);
          result /= L;
        } else if (order > 1) {
          result.setZero();
        }

        value_type t (u * curveLength_);
        value_type beta (curveLength_ / L);
        if (L == 0) beta = 1;
        value_type alpha (fabs(beta));
        if (forward_ == -1 && order%2 == 1) alpha *= -1;
        value_type c0 (initial_ [rzId_ + 0]), s0 (initial_ [rzId_ + 1]);
        value_type dx, dy, dtheta = 0;
        value_type c (cos (curvature_ * t)), s (sin (curvature_ * t));

        if (order <= 0) {
          std::ostringstream oss;
          oss << "order of derivative (" << order << ") should be positive.";
          throw std::runtime_error (oss.str ().c_str ());
        }
        if (order == 1) {
          dx = alpha * (c0 * c - s0 * s);
          dy = alpha * (c0 * s + s0 * c);
          dtheta = alpha * curvature_;
        } else {
          switch (order % 4) {
            case 2:
              dx =  alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * s + s0 * c);
              dy = -alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * c - s0 * s);
            case 3:
              dx = -alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * c - s0 * s);
              dy = -alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * s + s0 * c);
            case 0:
              dx =  alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * s + s0 * c);
              dy = -alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * c - s0 * s);
            case 1:
              dx =  alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * c - s0 * s);
              dy =  alpha * pow (curvature_ * beta, (value_type) (order - 1)) *
                (c0 * s + s0 * c);
          }
        }

        result [dxyId_ + 0] = dx;
        result [dxyId_ + 1] = dy;
        result [rzId_] = dtheta;
        // Express velocity in local frame
        Eigen::Matrix<value_type, 2, 2> R;
        R.col(0) << c0 * c - s0 * s, c0 * s + s0 * c;
        R.col(1) << - R(1,0), R(0,0);
        result.segment<2>(dxyId_) = R.transpose() * result.segment<2>(dxyId_);

        // Set wheel joint velocities
        for (std::vector<Wheels_t>::const_iterator w = wheels_.begin ();
             w < wheels_.end (); ++w) {
          result [w->j->rankInConfiguration ()] = 0;
        }
      }

      PathPtr_t ConstantCurvature::impl_extract
      (const interval_t& paramInterval) const throw (projection_error)
      {
        assert (!timeParameterization ());
        assert (timeRange ().second - timeRange ().first >= 0);
        value_type L (timeRange ().second - timeRange ().first);
        value_type tmin (paramInterval.first), tmax (paramInterval.second);
        value_type curveLength =0, pathLength = 0;
        if (L != 0) {
          curveLength = ((tmax - tmin)/L * curveLength_);
          pathLength = fabs (tmax - tmin)/L * length ();
        }
        Configuration_t init (robot_->configSize ());
        bool res (impl_compute (init, tmin)); assert (res);
        Configuration_t end (robot_->configSize ());
        res = impl_compute (end,tmax); assert (res);
        ConstantCurvaturePtr_t result (createCopy (weak_.lock ()));
        result->initial_ = init;
        result->end_ = end;
        result->curveLength_ = curveLength;
        result->timeRange (interval_t (0, pathLength));
        result->forward_ = curveLength > 0 ? 1 : -1;
        return result;
      }

      inline value_type meanBounds(const JointPtr_t& j, const size_type& i)
      {
        return (j->upperBound(i) + j->lowerBound(i))/2;
      }

      inline value_type saturate (const value_type& v, const JointPtr_t& j,
                                  const size_type& i)
      {
        return std::min(j->upperBound(i), std::max(j->lowerBound(i), v));
      }

      void ConstantCurvature::setWheelJoints
      (const JointPtr_t rz, const std::vector<JointPtr_t> wheels)
      {
        Transform3f zt (rz->currentTransformation ().inverse ());
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
          value_type rho (1./curvature_);
          for (std::vector<JointPtr_t>::const_iterator _wheels = wheels.begin();
               _wheels != wheels.end(); ++_wheels) {
            wheels_[rk].j = *_wheels;
            wheels_[rk].value = meanBounds(wheels_[rk].j, 0);
            const vector3_t wheelPos = zt.act
              (wheels_[rk].j->currentTransformation().translation());
            const value_type value (std::atan(wheelPos[0] /
                                              (rho - wheelPos[1])));
            wheels_[rk].value = saturate(meanBounds(wheels_[rk].j, 0) + value,
                                         *_wheels, 0);
            ++rk;
          }
        }
      }

    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
