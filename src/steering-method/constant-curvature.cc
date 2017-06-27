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

#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {

      ConstantCurvaturePtr_t ConstantCurvature::create
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type length, value_type curvature, size_type xyId, size_type rzId)
      {
        ConstantCurvature* ptr (new ConstantCurvature
                                (robot, init, end, length, curvature, xyId,
                                 rzId));
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
        os << "  length: " << forward_ * timeRange ().second << std::endl;
        os << "  curvature: " << curvature_ << std::endl;
        return os;
      }

      ConstantCurvature::ConstantCurvature
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type length, value_type curvature, size_type xyId, size_type rzId,
       ConstraintSetPtr_t constraints) :
        Path (std::make_pair (0., fabs (length)), robot->configSize (),
              robot->numberDof (), constraints), robot_ (robot),
        initial_ (init), end_ (end), curvature_ (curvature),
        xyId_ (xyId), rzId_ (rzId), forward_ (length > 0 ? 1 : -1)
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
        impl_compute (end_, timeRange ().second);
      }

      ConstantCurvature::ConstantCurvature
      (const DevicePtr_t& robot, ConfigurationIn_t init, ConfigurationIn_t end,
       value_type length, value_type curvature, size_type xyId,
       size_type rzId) :
        Path (std::make_pair (0., fabs (length)), robot->configSize (),
              robot->numberDof ()), robot_ (robot),
        initial_ (init), end_ (end), curvature_ (curvature), xyId_ (xyId),
        rzId_ (rzId), forward_ (length > 0 ? 1 : -1)
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
        impl_compute (end_, timeRange ().second);
      }

      ConstantCurvature::ConstantCurvature (const ConstantCurvature& other) :
        Path (other), robot_ (other.robot_), initial_ (other.initial_),
        end_ (other.end_), curvature_ (other.curvature_),  xyId_ (other.xyId_),
        rzId_ (other.rzId_), forward_ (other.forward_)
      {
      }

      ConstantCurvature::ConstantCurvature
      (const ConstantCurvature& other, const ConstraintSetPtr_t& constraints) :
        parent_t (other, constraints), robot_ (other.robot_),
        initial_ (other.initial_), end_ (other.end_),
        curvature_ (other.curvature_),  xyId_ (other.xyId_),
        rzId_ (other.rzId_), forward_ (other.forward_)
      {
      }

      bool ConstantCurvature::impl_compute (ConfigurationOut_t result,
                                            value_type param) const
      {
        // Does a linear interpolation on all the joints.
        const value_type u = (timeRange ().second == 0) ? 0 :
          param/timeRange ().second;
        model::interpolate (robot_, initial_, end_, u, result);

        value_type t (forward_ * param);
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
        value_type t (forward_ * param);
        value_type alpha (1);
        if (forward_ == -1 && order%2 == 1) alpha = -1;
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
        } else if (order % 4 == 2) {
          dx =  alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * s + s0 * c);
          dy = -alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * c - s0 * s);
        } else if (order % 4 == 3) {
          dx = -alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * c - s0 * s);
          dy = -alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * s + s0 * c);
        } else if (order % 4 == 0) {
          dx =  alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * s + s0 * c);
          dy = -alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * c - s0 * s);
        } else if (order % 4 == 1) {
          dx =  alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * c - s0 * s);
          dy =  alpha * pow (curvature_, (value_type) (order - 1)) *
            (c0 * s + s0 * c);
        }

        result [dxyId_ + 0] = dx;
        result [dxyId_ + 1] = dy;
        result [rzId_] = dtheta;

        // Set wheel joint velocities
        for (std::vector<Wheels_t>::const_iterator w = wheels_.begin ();
             w < wheels_.end (); ++w) {
          result [w->j->rankInConfiguration ()] = 0;
        }
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
#ifdef PINOCCHIO
        Transform3f zt (rz->currentTransformation ().inverse ());
#else
        Transform3f zt (rz->currentTransformation ());
        zt.inverse();
#endif
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
            const vector3_t wheelPos = zt.transform
              (wheels_[rk].j->currentTransformation().getTranslation());
#ifdef PINOCCHIO
            const value_type value (std::atan(wheelPos[2] /
                                              (- wheelPos[1] - rho)));
#else
            const value_type value (std::atan(wheelPos[0] /
                                              (rho - wheelPos[1])));
#endif
            wheels_[rk].value = saturate(meanBounds(wheels_[rk].j, 0) + value,
                                         *_wheels, 0);
            ++rk;
          }
        }
      }

    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
