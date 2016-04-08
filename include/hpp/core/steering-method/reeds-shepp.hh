//
// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH
# define HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/model/joint.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/reeds-shepp-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates ReedsSheppPath instances
      ///
      class HPP_CORE_DLLAPI ReedsShepp : public SteeringMethod
      {
        public:
          /// Create instance and return shared pointer
          static ReedsSheppPtr_t create (const ProblemPtr_t& problem)
          {
            ReedsShepp* ptr = new ReedsShepp (problem);
            ReedsSheppPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }
          /// Copy instance and return shared pointer
          static ReedsSheppPtr_t createCopy
            (const ReedsSheppPtr_t& other)
            {
              ReedsShepp* ptr = new ReedsShepp (*other);
              ReedsSheppPtr_t shPtr (ptr);
              ptr->init (shPtr);
              return shPtr;
            }
          /// Copy instance and return shared pointer
          virtual SteeringMethodPtr_t copy () const
          {
            return createCopy (weak_.lock ());
          }

          /// create a path between two configurations
          virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
              ConfigurationIn_t q2) const
          {
            ReedsSheppPathPtr_t path =
              ReedsSheppPath::create (device_.lock (), q1, q2,
                  rho_ , xy_->rankInConfiguration(),
                  rz_->rankInConfiguration(), constraints ());
            if (lw_ && rw_) path->setWheelJoints (lw_, rw_, alphaPrime_);
            if (sa_) path->setSteeringJoint (sa_);
            return path;
          }
        protected:
          /// Constructor with robot
          /// Weighed distance is created from robot
          ReedsShepp (const ProblemPtr_t& problem) :
            SteeringMethod (problem), device_ (problem->robot ()),
            rho_ (1.), weak_ ()
        {
          computeDistanceBetweenAxes ();
        }

          /// Copy constructor
          ReedsShepp (const ReedsShepp& other) :
            SteeringMethod (other), device_ (other.device_),
            rho_ (other.rho_)
        {
        }

          /// Store weak pointer to itself
          void init (ReedsSheppWkPtr_t weak)
          {
            core::SteeringMethod::init (weak);
            weak_ = weak;
          }
        private:
          inline void computeAngles() 
          {
            const Transform3f& zt (rz_->currentTransformation ());
            const Transform3f& lt (lw_->currentTransformation ());
            const Transform3f& rt (rw_->currentTransformation ());
            const vector3_t& zp (zt.getTranslation ());
            const vector3_t& lp (lt.getTranslation ());
            const vector3_t& rp (rt.getTranslation ());
            value_type beta = (zp - (lp + rp) / 2).norm();
            value_type delta = (lp - rp).norm();
            alpha_ = (lw_->upperBound(0) - lw_->lowerBound(0)) / 2;
            rho_ = delta / 2 + beta / std::tan(alpha_);
            alphaPrime_ = std::atan2(beta, rho_ + delta/2);
          }

          inline void computeAngle() 
          {
            const Transform3f& zt (rz_->currentTransformation ());
            const Transform3f& st (sa_->currentTransformation ());
            const vector3_t& zp (zt.getTranslation ());
            const vector3_t& sp (st.getTranslation ());
            value_type beta = (zp - sp).norm();
            alpha_ = (lw_->upperBound(0) - lw_->lowerBound(0)) / 2;
            rho_ = beta / std::tan(alpha_);
          }

          void computeDistanceBetweenAxes ()
          {
            DevicePtr_t d (device_.lock());
            sa_ = NULL; lw_ = NULL; rw_ = NULL;
            rho_ = 1;
            xy_ = d->rootJoint ();
            // Test that kinematic chain is as expected
            if (!dynamic_cast <model::JointTranslation <2>* > (xy_)) {
              throw std::runtime_error ("root joint should be of type "
                  "model::JointTranslation <2>");
            }
            if (xy_->numberChildJoints () != 1) {
              throw std::runtime_error ("Root joint should have one child");
            }
            rz_ = xy_->childJoint (0);
            if (!dynamic_cast <model::jointRotation::UnBounded*> (rz_)) {
              throw std::runtime_error ("second joint should be of type "
                  "model::jointRotation::Unbounded");
            }
            try {
              const model::JointGroup_t& g = d->jointGroup ("reedsshepp_wheels");
              switch (g.size()) {
                case 1: {
                          sa_ = g[0];
                          computeAngle ();
                        }
                        break;
                case 2: {
                          lw_ = g[0]; // Left wheel first
                          rw_ = g[1];
                          computeAngles ();
                        }
                        break;
                default:
                        break;
              }
            } catch (const std::runtime_error&) {
              throw std::runtime_error ("A group named \"reedsshepp_wheels\" must be defined in the SRDF");
            }
            hppDout (info, "rho_ = " << rho_);
          }

          DeviceWkPtr_t device_;
          // distance between front and rear wheel axes.
          value_type rho_, alpha_, alphaPrime_;
          JointPtr_t xy_, rz_, sa_, lw_, rw_;
          ReedsSheppWkPtr_t weak_;
      }; // ReedsShepp
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH
