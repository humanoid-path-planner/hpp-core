// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/constraints/svd.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>

namespace hpp {
namespace core {
namespace steeringMethod {
template <int _PB, int _SO>
PathPtr_t Spline<_PB, _SO>::impl_compute(ConfigurationIn_t q1,
                                         ConfigurationIn_t q2) const {
  enum { NDerivativeConstraintPerSide = int((SplineOrder + 1 - 2) / 2) };
  typedef Eigen::Matrix<value_type, Eigen::Dynamic,
                        NDerivativeConstraintPerSide>
      DerMatrix_t;
  typedef typename DerMatrix_t::ConstantReturnType DefaultDerivatives_t;

  DefaultDerivatives_t defaultDer(DerMatrix_t::Zero(
      device_.lock()->numberDof(), NDerivativeConstraintPerSide));
  std::vector<int> orders(NDerivativeConstraintPerSide);
  for (std::size_t i = 0; i < NDerivativeConstraintPerSide; ++i)
    orders[i] = int(i + 1);
  return impl_compute(q1, orders, defaultDer, q2, orders, defaultDer, -1);
}

template <int _PB, int _SO>
PathPtr_t Spline<_PB, _SO>::steer(ConfigurationIn_t q1, std::vector<int> order1,
                                  matrixIn_t derivatives1, ConfigurationIn_t q2,
                                  std::vector<int> order2,
                                  matrixIn_t derivatives2,
                                  value_type length) const {
  // Check the size of the derivatives.
  assert(q1.size() == device_.lock()->configSize());
  assert(q1.size() == q2.size());
  assert(derivatives1.rows() == device_.lock()->numberDof());
  assert(derivatives2.rows() == device_.lock()->numberDof());
  return impl_compute(q1, order1, derivatives1, q2, order2, derivatives2,
                      length);
}

template <int _PB, int _SO>
template <typename Derived>
PathPtr_t Spline<_PB, _SO>::impl_compute(
    ConfigurationIn_t q1, std::vector<int> order1,
    const Eigen::MatrixBase<Derived>& derivatives1, ConfigurationIn_t q2,
    std::vector<int> order2, const Eigen::MatrixBase<Derived>& derivatives2,
    value_type length) const {
  // Compute the decomposition
  // typedef Eigen::Matrix<value_type, SplineOrder+1, SplineOrder+1>
  // ConstraintMatrix_t;
  typedef Eigen::Matrix<value_type, Eigen::Dynamic, SplineOrder + 1,
                        Eigen::RowMajor>
      ConstraintMatrix_t;
  typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic,
                        Eigen::RowMajor>
      RhsMatrix_t;

  if (length <= 0) {
    DistancePtr_t d = problem()->distance();
    length = (*d)(q1, q2);
  }
  SplinePathPtr_t p =
      SplinePath::create(device_.lock(), interval_t(0, length), constraints());

  const size_type nbConstraints = 2 + derivatives1.cols() + derivatives2.cols();
  ConstraintMatrix_t coeffs(nbConstraints, SplineOrder + 1);
  RhsMatrix_t rhs(nbConstraints, device_.lock()->numberDof());

  p->base(q1);  // TODO use the center ?
  // p->base(device_.lock()->neutralConfiguration()); // TODO use the center ?
  // Configuration_t qmiddle (q1.size());
  // pinocchio::interpolate<pinocchio::RnxSOnLieGroupMap>(device_.lock(), q1,
  // q2, 0.5, qmiddle); p->base(qmiddle);

  // Compute the matrices
  // TODO calls to basisFunctionDerivative could be cached as they do not
  // depend on the inputs.
  p->basisFunctionDerivative(0, 0, coeffs.row(0).transpose());
  pinocchio::difference<pinocchio::RnxSOnLieGroupMap>(device_.lock(), q1,
                                                      p->base(), rhs.row(0));
  for (std::size_t i = 0; i < order1.size(); ++i)
    p->basisFunctionDerivative(order1[i], 0,
                                                coeffs.row(i + 1).transpose());
  rhs.middleRows(1, order1.size()).transpose() = derivatives1;

  size_type row = 1 + order1.size();
  p->basisFunctionDerivative(0, 1,
                                              coeffs.row(row).transpose());
  pinocchio::difference<pinocchio::RnxSOnLieGroupMap>(device_.lock(), q2,
                                                      p->base(), rhs.row(row));
  ++row;
  for (std::size_t i = 0; i < order2.size(); ++i)
    p->basisFunctionDerivative(
        order2[i], 1, coeffs.row(i + row).transpose());
  rhs.middleRows(row, order2.size()).transpose() = derivatives2;

  // Solve the problem
  // coeffs * P = rhs
  typedef Eigen::JacobiSVD<ConstraintMatrix_t> SVD_t;
  SVD_t svd(coeffs, Eigen::ComputeFullU | Eigen::ComputeFullV);
  p->parameters(svd.solve(rhs));

  return p;
}

template <int _PB, int _SO>
Spline<_PB, _SO>::Spline(const ProblemConstPtr_t& problem)
    : SteeringMethod(problem), device_(problem->robot()) {}

/// Copy constructor
template <int _PB, int _SO>
Spline<_PB, _SO>::Spline(const Spline& other)
    : SteeringMethod(other), device_(other.device_) {}

// template class Spline<path::CanonicalPolynomeBasis, 1>; // equivalent to
// StraightPath template class Spline<path::CanonicalPolynomeBasis, 2>; template
// class Spline<path::CanonicalPolynomeBasis, 3>;
template class Spline<path::BernsteinBasis, 1>;  // equivalent to StraightPath
// template class Spline<path::BernsteinBasis, 2>;
template class Spline<path::BernsteinBasis, 3>;
template class Spline<path::BernsteinBasis, 5>;
}  // namespace steeringMethod
}  // namespace core
}  // namespace hpp
