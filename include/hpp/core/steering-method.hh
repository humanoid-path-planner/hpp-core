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

#ifndef HPP_CORE_STEERING_METHOD_HH
# define HPP_CORE_STEERING_METHOD_HH

# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Steering method
    ///
    /// A steering method creates paths between pairs of
    /// configurations for a robot.  They are usually used to take
    /// into account nonholonomic constraints of some robots
    class HPP_CORE_DLLAPI SteeringMethod
    {
    public:
      /// create a path between two configurations
      PathPtr_t operator() (ConfigurationIn_t q1,
			    ConfigurationIn_t q2) const
      {
	PathPtr_t path (impl_compute (q1, q2));
	path->constraints (constraints_);
	return path;
      }
      /// \name Constraints applicable to the robot
      /// \{

      /// Set constraint set
      void constraints (const ConstraintSetPtr_t& constraints)
      {
	constraints_ = constraints;
      }

      ///Get constraint set
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }
      /// \}

    protected:
      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const = 0;
    private:
      /// Set of constraints to apply on the paths produced
      ConstraintSetPtr_t constraints_;
    }; // class SteeringMethod
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_HH
