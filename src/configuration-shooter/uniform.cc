//
// Copyright (c) 2018 CNRS
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

# include <hpp/core/configuration-shooter/uniform.hh>

# include <pinocchio/algorithm/joint-configuration.hpp>

# include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {

      void Uniform::shoot (Configuration_t& config) const
      {
        size_type extraDim = robot_->extraConfigSpace ().dimension ();
        size_type offset = robot_->configSize () - extraDim;

        config.resize(robot_->configSize ());
        config.head (offset) = ::pinocchio::randomConfiguration(robot_->model());

        if(sampleExtraDOF_){
            // Shoot extra configuration variables
            for (size_type i=0; i<extraDim; ++i) {
              value_type lower = robot_->extraConfigSpace ().lower (i);
              value_type upper = robot_->extraConfigSpace ().upper (i);
              value_type range = upper - lower;
              if ((range < 0) ||
                  (range == std::numeric_limits<double>::infinity())) {
                std::ostringstream oss
                  ("Cannot uniformy sample extra config variable ");
                oss << i << ". min = " <<lower<< ", max = " << upper << std::endl;
                throw std::runtime_error (oss.str ());
              }
              config [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
            }
        }else{
            config.tail(extraDim).setZero();
        }
      }

    } // namespace configurationShooter
  } // namespace core
} // namespace hpp
