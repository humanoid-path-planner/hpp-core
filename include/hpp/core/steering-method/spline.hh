//
// Copyright (c) 2017 CNRS
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

#ifndef HPP_CORE_STEERING_METHOD_SPLINE_HH
# define HPP_CORE_STEERING_METHOD_SPLINE_HH

# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/steering-method.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Steering method that creates path::Spline instances
      ///
      template <int _PolynomeBasis, int _SplineOrder>
      class HPP_CORE_DLLAPI Spline : public SteeringMethod
      {
        public:
          enum {
            PolynomeBasis = _PolynomeBasis,
            SplineOrder = _SplineOrder
          };
          typedef path::Spline<PolynomeBasis, SplineOrder> SplinePath;
          typedef typename SplinePath::Ptr_t SplinePathPtr_t;

          typedef boost::shared_ptr<Spline> Ptr_t;
          typedef boost::weak_ptr<Spline> WkPtr_t;

          static Ptr_t create (const Problem& problem)
          {
            Spline* ptr = new Spline (problem);
            Ptr_t shPtr (ptr);
            ptr->init (shPtr);
            return shPtr;
          }

          /// Copy instance and return shared pointer
          static Ptr_t createCopy (const Ptr_t& other)
          {
            Spline* ptr = new Spline (*other);
            Ptr_t shPtr (ptr);
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
              ConfigurationIn_t q2) const;

          /// create a path between two configurations
          PathPtr_t steer (ConfigurationIn_t q1, std::vector<int> order1, matrixIn_t derivatives1,
                           ConfigurationIn_t q2, std::vector<int> order2, matrixIn_t derivatives2) const;

        protected:
          /// Constructor
          Spline (const Problem& problem);

          /// Copy constructor
          Spline (const Spline& other);

          /// Store weak pointer to itself
          void init (WkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          /// create a path between two configurations
          template <typename Derived>
          PathPtr_t impl_compute (
              ConfigurationIn_t q1, std::vector<int> order1,  const Eigen::MatrixBase<Derived>& derivatives1,
              ConfigurationIn_t q2, std::vector<int> order2,  const Eigen::MatrixBase<Derived>& derivatives2) const;

          DeviceWkPtr_t device_;
          WkPtr_t weak_;
      }; // Spline
      /// \}
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_REEDS_SHEPP_HH
