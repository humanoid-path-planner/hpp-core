//
// Copyright (c) 2017 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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

          typedef shared_ptr<Spline> Ptr_t;
          typedef weak_ptr<Spline> WkPtr_t;

          static Ptr_t create (const ProblemConstPtr_t& problem)
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
          Spline (const ProblemConstPtr_t& problem);

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
