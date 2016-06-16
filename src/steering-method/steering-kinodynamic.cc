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

# include <hpp/core/steering-method/steering-kinodynamic.hh>

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>

# include <hpp/core/kinodynamic-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      PathPtr_t Kinodynamic::impl_compute (ConfigurationIn_t q1,
                                           ConfigurationIn_t q2) const
      {
        value_type length = (*problem_->distance()) (q1, q2);        
        KinodynamicPathPtr_t path = KinodynamicPath::create (device_.lock (), q1, q2,length/* ... */);
        // TODO
        return path;
      }
      
      
      
      
      
      Kinodynamic::Kinodynamic (const ProblemPtr_t& problem) :
        SteeringMethod (problem), device_ (problem->robot ()), weak_ ()
      {
      }
      
      /// Copy constructor
      Kinodynamic::Kinodynamic (const Kinodynamic& other) :
        SteeringMethod (other), device_ (other.device_)
      {
      }
      
      
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
