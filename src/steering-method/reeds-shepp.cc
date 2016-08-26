// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/core/steering-method/reeds-shepp.hh>

#include <pinocchio/multibody/joint/joint.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/reeds-shepp-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      PathPtr_t ReedsShepp::impl_compute (ConfigurationIn_t q1,
          ConfigurationIn_t q2) const
      {
        ReedsSheppPathPtr_t path =
          ReedsSheppPath::create (device_.lock (), q1, q2,
              rho_ , xy_, rz_, constraints ());
        path->setWheelJoints (turningJoint_, wheels_);
        return path;
      }

      void ReedsShepp::computeRadius ()
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

      ReedsShepp::ReedsShepp (const ProblemPtr_t& problem) :
        SteeringMethod (problem), device_ (problem->robot ()),
        rho_ (1.), xy_ (0), rz_(2), weak_ ()
      {
        DevicePtr_t d (device_.lock());
        std::string sn = d->rootJoint()->jointModel().shortname();
        if (sn == se3::JointModelPlanar::classname()) {
          turningJoint_ = d->rootJoint();
        // } else if (sn == se3::JointModelPlanar::classname() && d->rootJoint()->configSize() == 2) {
          // std::string sn2 = d->getJointAtConfigRank(2)->jointModel().shortname();
          // if (sn2 == "revoluteunbounded")
            // turningJoint_ = d->getJointAtConfigRank(2);
          // else
            // throw std::runtime_error ("root joint should be of type "
                // "se3::JointModelPlanar or JointModelTranslation + JointModelRevolute");
        } else {
          throw std::runtime_error ("root joint should be of type "
              "se3::JointModelPlanar" /*" or JointModelTranslation + JointModelRevolute"*/);
        }

        guessWheels();
        computeRadius();
      }

      ReedsShepp::ReedsShepp (const ProblemPtr_t& problem,
          const value_type turningRadius,
          JointPtr_t xyJoint, JointPtr_t rzJoint,
          std::vector <JointPtr_t> wheels) :
        SteeringMethod (problem), device_ (problem->robot ()),
        rho_ (turningRadius),
        turningJoint_ (rzJoint),
        xy_ (xyJoint->rankInConfiguration()),
        rz_ (rzJoint->rankInConfiguration()),
        wheels_ (wheels), weak_ ()
      {
      }

      /// Copy constructor
      ReedsShepp::ReedsShepp (const ReedsShepp& other) :
        SteeringMethod (other), device_ (other.device_),
        rho_ (other.rho_)
      {
      }

      inline value_type ReedsShepp::computeAngle(const JointPtr_t wheel) const
      {
        // Compute wheel position in joint RZ
        const Transform3f zt (turningJoint_->currentTransformation ());
        const Transform3f wt (zt.actInv (wheel->currentTransformation ()));
        const vector3_t& wp (wt.translation ());

        // Assume the non turning wheels are on the plane z = 0 of 
        // in joint rz.
        const value_type alpha = (wheel->upperBound(0) - wheel->lowerBound(0)) / 2;
        const value_type delta = std::abs(wp[1]);
        const value_type beta = std::abs(wp[2]);

        return delta + beta / std::tan(alpha);
      }

      inline void ReedsShepp::guessWheels()
      {
        wheels_.clear();
        for (std::size_t i = 0; i < turningJoint_->numberChildJoints(); ++i) {
          JointPtr_t j = turningJoint_->childJoint(i);
          if (j->configSize() != 1) continue;
          if (!j->isBounded(0))     continue;
          if (j->name().find("wheel") == std::string::npos) continue;
          wheels_.push_back(j);
        }
      }
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
