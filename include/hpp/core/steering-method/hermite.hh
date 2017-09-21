// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_HERMITE_HH
# define HPP_CORE_STEERING_METHOD_HERMITE_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/steering-method/fwd.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/path/hermite.hh>
# include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates path::Hermite instances
      ///
      class HPP_CORE_DLLAPI Hermite : public SteeringMethod
      {
        public:
          /// Create instance and return shared pointer
          static HermitePtr_t create  (const Problem& problem)
          {
            Hermite* ptr = new Hermite (problem);
            HermitePtr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static HermitePtr_t createCopy (const HermitePtr_t& other)
          {
            Hermite* ptr = new Hermite (*other);
            HermitePtr_t shPtr (ptr);
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
            path::HermitePtr_t path = path::Hermite::create
              (problem_.robot(), q1, q2, constraints ());

            path->computeHermiteLength();
            return path;
          }

        protected:
          /// Constructor with weighed distance
          Hermite (const Problem& problem) :
            SteeringMethod (problem), weak_ ()
          {}

          /// Copy constructor
          Hermite (const Hermite& other) :
            SteeringMethod (other), weak_ ()
          {}

          /// Store weak pointer to itself
          void init (HermiteWkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          HermiteWkPtr_t weak_;
      }; // Hermite
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_HERMITE_HH
