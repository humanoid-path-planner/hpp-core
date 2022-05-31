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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH

#include <hpp/constraints/explicit-constraint-set.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/path-optimization/spline-gradient-based-abstract.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/core/steering-method/spline.hh>

namespace hpp {
namespace core {
/// \addtogroup path_optimization
/// \{
namespace pathOptimization {
template <int _PolynomeBasis, int _SplineOrder>
class HPP_CORE_DLLAPI SplineGradientBased
    : public SplineGradientBasedAbstract<_PolynomeBasis, _SplineOrder> {
 public:
  typedef SplineGradientBasedAbstract<_PolynomeBasis, _SplineOrder> Base;
  enum { PolynomeBasis = _PolynomeBasis, SplineOrder = _SplineOrder };
  typedef shared_ptr<SplineGradientBased> Ptr_t;

  using typename Base::Spline;
  using typename Base::SplinePtr_t;
  using typename Base::Splines_t;

  /// Return shared pointer to new object.
  /// Default cost is path length.
  static Ptr_t create(const ProblemConstPtr_t& problem);

  /// Optimize path
  /// \li 1) Transform straight paths into splines
  /// \li 2) Add continuity constraints
  /// \li 3) Add problem constraints
  /// \li 4) Make cost function
  /// \li 5) Compute explicit representation of linear constraints.
  /// \li 6)
  /// \li - 6.1) Compute cost hessian
  /// \li - 6.2) Compute optimum
  /// \li - 6.3) Check path for collision. If no collision, return solution
  /// \li - 6.4) add collision constraint
  /// \li - 6.5) re-compute explicit representation of linear constraints.
  /// \li 7) - Build result path.
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

 protected:
  using Base::problem;
  using Base::robot_;
  using typename Base::RowBlockIndices;
  using typename Base::SplineOptimizationData;
  using typename Base::SplineOptimizationDatas_t;

  SplineGradientBased(const ProblemConstPtr_t& problem);

  /// \name Constraint creation
  /// \{

  /// Compute a conservative linear representation of the constraints.
  ///
  /// It determines:
  /// \li which DoFs can be computed explicitely. These DoFs are removed
  ///     from the variables and computed explicitely.
  /// \li which DoFs are constrained implicitely. These DoFs are removed
  ///     from the variables and are set constant.
  /// \li which DoFs are not constrained. These DoFs are kept as
  ///     variables for optimization.
  virtual void addProblemConstraints(const PathVectorPtr_t& init,
                                     const Splines_t& splines,
                                     LinearConstraint& lc,
                                     SplineOptimizationDatas_t& sods) const;

  void addProblemConstraintOnPath(const PathPtr_t& path,
                                  const size_type& idxSpline,
                                  const SplinePtr_t& spline,
                                  LinearConstraint& lc,
                                  SplineOptimizationData& sod) const;

  /// \param guessThr Threshold used to check whether the Jacobian
  ///                 contains rows of zeros, in which case the
  ///                 corresponding DoF is considered passive.
  Eigen::RowBlockIndices computeActiveParameters(
      const PathPtr_t& path, const constraints::solver::BySubstitution& hs,
      const value_type& guessThr = -1,
      const bool& useExplicitInput = false) const;

  /// \}

  bool checkOptimum_;

 private:
  typedef typename Base::Reports_t Reports_t;
  struct CollisionFunctions;

  void addCollisionConstraint(const std::size_t idxSpline,
                              const SplinePtr_t& spline,
                              const SplinePtr_t& nextSpline,
                              const SplineOptimizationData& sod,
                              const PathValidationReportPtr_t& report,
                              LinearConstraint& collision,
                              CollisionFunctions& functions) const;

  bool findNewConstraint(LinearConstraint& constraint,
                         LinearConstraint& collision,
                         LinearConstraint& collisionReduced,
                         CollisionFunctions& functions, const std::size_t iF,
                         const SplinePtr_t& spline,
                         const SplineOptimizationData& sod) const;

  template <typename Cost_t>
  bool checkHessian(const Cost_t& cost, const matrix_t& H,
                    const Splines_t& splines) const;

  // Continuity constraints
  // matrix_t Jcontinuity_;
  // vector_t rhsContinuity_;
};  // GradientBased
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
