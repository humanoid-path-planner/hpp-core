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

# include <hpp/util/debug.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>
# include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Steering method
    ///
    /// A steering method creates paths between pairs of
    /// configurations for a robot. They are usually used to take
    /// into account nonholonomic constraints of some robots
    class HPP_CORE_DLLAPI SteeringMethod
    {
    public:
      /// create a path between two configurations
      /// \return a Path from q1 to q2 if found. An empty
      /// Path if Path could not be built.
      PathPtr_t operator() (ConfigurationIn_t q1,
			    ConfigurationIn_t q2) const
      {
        try {
          return impl_compute (q1, q2);
        } catch (const projection_error& e) {
          hppDout (info, "Could not build path: " << e.what());
        }
	return PathPtr_t ();
      }

      /// Copy instance and return shared pointer
      virtual SteeringMethodPtr_t copy () const = 0;

      /// \name Constraints applicable to the robot.
      /// These constraints are not automatically taken into
      /// account. Child class can use it if they need.
      /// \{

      /// Set constraint set
      void constraints (const ConstraintSetPtr_t& constraints)
      {
	constraints_ = constraints;
      }

      /// Get constraint set
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }
      /// \}

    protected:
      /// Constructor
      SteeringMethod (ProblemPtr_t problem) :
        problem_ (problem), constraints_ (), weak_ ()
      {
      }
      /// Copy constructor
      ///
      /// Constraints are copied
      SteeringMethod (const SteeringMethod& other) :
        problem_ (other.problem_), constraints_ (), weak_ ()
	{
	  if (other.constraints_) {
	    constraints_ = HPP_DYNAMIC_PTR_CAST (ConstraintSet,
						 other.constraints_->copy ());
	  }
	}
      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const = 0;
      /// Store weak pointer to itself.
      void init (SteeringMethodWkPtr_t weak)
      {
	weak_ = weak;
      }

      ProblemPtr_t problem_;

    private:
      /// Set of constraints to apply on the paths produced
      ConstraintSetPtr_t constraints_;
      /// Weak pointer to itself
      SteeringMethodWkPtr_t weak_;
    }; // class SteeringMethod
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_HH
