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

#include <hpp/core/distance.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/reeds-shepp-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      PathPtr_t ReedsShepp::impl_compute (ConfigurationIn_t q1,
          ConfigurationIn_t q2) const
      {
        // TODO this should not be done here.
        // See todo in class ConstantCurvature
        Configuration_t qEnd (q2);
        qEnd.segment<2>(xyId_) = q1.segment<2>(xyId_);
        qEnd.segment<2>(rzId_) = q1.segment<2>(rzId_);
        // The length corresponding to the non RS DoF
        value_type extraL = (*problem().distance()) (q1, qEnd);

        ReedsSheppPathPtr_t path =
          ReedsSheppPath::create (device_.lock (), q1, q2, extraL,
                                  rho_ , xyId_, rzId_, wheels_, constraints ());
        return path;
      }

      ReedsShepp::ReedsShepp (const Problem& problem) :
        CarLike (problem), weak_ ()
      {
      }

      ReedsShepp::ReedsShepp (const Problem& problem,
          const value_type turningRadius,
          JointPtr_t xyJoint, JointPtr_t rzJoint,
          std::vector <JointPtr_t> wheels) :
	CarLike (problem, turningRadius, xyJoint, rzJoint, wheels)
      {
      }

      /// Copy constructor
      ReedsShepp::ReedsShepp (const ReedsShepp& other) :
        CarLike (other)
      {
      }

    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
