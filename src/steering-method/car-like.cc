// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/car-like.hh>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/joint/joint.hpp>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      CarLike::CarLike (const Problem& problem) :
	SteeringMethod (problem), device_ (problem.robot ()), rho_ (1.),
        xyId_ (0), rzId_ (2)
      {
        DevicePtr_t d (device_.lock());
        xy_ = d->getJointAtConfigRank(0);
        rz_ = d->getJointAtConfigRank(2);
        guessWheels();
        computeRadius();
      }

      CarLike::CarLike (const Problem& problem,
			const value_type turningRadius,
			JointPtr_t xyJoint, JointPtr_t rzJoint,
			std::vector <JointPtr_t> wheels) :
        SteeringMethod (problem), device_ (problem.robot ()),
        rho_ (turningRadius), xy_ (xyJoint), rz_ (rzJoint),
        xyId_ (xy_->rankInConfiguration ()), wheels_ (wheels), weak_ ()
      {
        if (rz_->jointModel ().classname () == "JointModelPlanar") {
          rzId_ = rz_->rankInConfiguration () + 2;
        } else {
          rzId_ = rz_->rankInConfiguration ();
        }
      }

      /// Copy constructor
      CarLike::CarLike (const CarLike& other) :
        SteeringMethod (other), device_ (other.device_),
        rho_ (other.rho_), xy_ (other.xy_), rz_ (other.rz_),
        xyId_ (other.xyId_), rzId_ (other.rzId_)
      {
      }

      void CarLike::computeRadius ()
      {
        rho_ = 1;
        if (wheels_.empty()) return;
        rho_ = 0;
        for (std::vector<JointPtr_t>::const_iterator _wheels = wheels_.begin();
            _wheels != wheels_.end(); ++_wheels) {
          rho_ = std::max (rho_, computeAngle(*_wheels));
        }
        hppDout (info, "rho_ = " << rho_);
      }

      inline value_type CarLike::computeAngle(const JointPtr_t wheel) const
      {
        // Compute wheel position in joint RZ
        const Transform3f zt (rz_->currentTransformation ());
        const Transform3f wt (zt.inverse () * wheel->currentTransformation ());
        const vector3_t& wp (wt.translation ());

        // Assume the non turning wheels are on the plane z = 0 of
        // in joint rz.
        const value_type alpha = (wheel->upperBound(0) - wheel->lowerBound(0)) / 2;
        const value_type delta = std::abs(wp[1]);
        const value_type beta = std::abs(wp[0]);

        return delta + beta / std::tan(alpha);
      }

      inline void CarLike::guessWheels()
      {
        wheels_.clear();
        for (std::size_t i = 0; i < rz_->numberChildJoints(); ++i) {
          JointPtr_t j = rz_->childJoint(i);
          if (j->configSize() != 1) continue;
          if (!j->isBounded(0))     continue;
          if (j->name().find("wheel") == std::string::npos) continue;
          wheels_.push_back(j);
        }
      }

    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
