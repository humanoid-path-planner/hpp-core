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

#ifndef HPP_CORE_STEERING_METHOD_STRAIGHT_HH
# define HPP_CORE_STEERING_METHOD_STRAIGHT_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/steering-method/fwd.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/distance.hh>
# include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates StraightPath instances
      ///
      class HPP_CORE_DLLAPI Straight : public SteeringMethod
      {
        public:
          /// Create instance and return shared pointer
          static StraightPtr_t create (const Problem& problem)
          {
            Straight* ptr = new Straight (problem);
            StraightPtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }
          /// Copy instance and return shared pointer
          static StraightPtr_t createCopy
            (const StraightPtr_t& other)
            {
              Straight* ptr = new Straight (*other);
              StraightPtr_t shPtr (ptr);
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
            value_type length = (*problem_.distance()) (q1, q2);
            ConstraintSetPtr_t c;
            if (constraints() && constraints()->configProjector ()) {
              c = HPP_STATIC_PTR_CAST (ConstraintSet, constraints()->copy ());
              c->configProjector()->rightHandSideFromConfig (q1); 
            } else {
              c = constraints ();
            }
            PathPtr_t path = StraightPath::create
              (problem_.robot(), q1, q2, length, c);
            return path;
          }
        protected:
          /// Constructor with robot
          /// Weighed distance is created from robot
          Straight (const Problem& problem) :
            SteeringMethod (problem), weak_ ()
        {
        }
          /// Copy constructor
          Straight (const Straight& other) :
            SteeringMethod (other), weak_ ()
        {
        }

          /// Store weak pointer to itself
          void init (StraightWkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:

          StraightWkPtr_t weak_;
      }; // Straight
    }
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_STRAIGHT_HH
