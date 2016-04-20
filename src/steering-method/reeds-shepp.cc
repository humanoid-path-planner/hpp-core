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

# include <hpp/core/steering-method/reeds-shepp.hh>

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>

# include <hpp/core/problem.hh>
# include <hpp/core/reeds-shepp-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      PathPtr_t ReedsShepp::impl_compute (ConfigurationIn_t q1,
          ConfigurationIn_t q2) const
      {
        ReedsSheppPathPtr_t path =
          ReedsSheppPath::create (device_.lock (), q1, q2,
              rho_ , xy_->rankInConfiguration(),
              rz_->rankInConfiguration(), constraints ());
        path->setWheelJoints (rz_, wheels_);
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
        rho_ (1.), weak_ ()
      {
        DevicePtr_t d (device_.lock());
        xy_ = d->getJointAtConfigRank(0);
        if (!dynamic_cast <model::JointTranslation <2>* > (xy_)) {
          throw std::runtime_error ("root joint should be of type "
              "model::JointTranslation <2>");
        }
        rz_ = d->getJointAtConfigRank(2);
        if (!dynamic_cast <model::jointRotation::UnBounded*> (rz_)) {
          throw std::runtime_error ("second joint should be of type "
              "model::jointRotation::Unbounded");
        }

        guessWheels();
        computeRadius();
      }

      ReedsShepp::ReedsShepp (const ProblemPtr_t& problem,
          const value_type turningRadius,
          JointPtr_t xyJoint, JointPtr_t rzJoint,
          std::vector <JointPtr_t> wheels) :
        SteeringMethod (problem), device_ (problem->robot ()),
        rho_ (turningRadius), xy_ (xyJoint), rz_ (rzJoint),
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
        const Transform3f zt (rz_->currentTransformation ());
        const Transform3f wt (zt.inverseTimes (wheel->currentTransformation ()));
        const vector3_t& wp (wt.getTranslation ());

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
