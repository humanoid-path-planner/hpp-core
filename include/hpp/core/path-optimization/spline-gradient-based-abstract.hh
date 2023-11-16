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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_ABSTRACT_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_ABSTRACT_HH

#include <hpp/constraints/explicit-constraint-set.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/path-optimization/linear-constraint.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path/spline.hh>
#include <hpp/core/steering-method/spline.hh>

namespace hpp {
namespace core {
/// \addtogroup path_optimization
/// \{
namespace pathOptimization {
/// Common base for optimization-based path optimizer with splines.
template <int _PolynomeBasis, int _SplineOrder>
class HPP_CORE_DLLAPI SplineGradientBasedAbstract : public PathOptimizer {
 public:
  enum { PolynomeBasis = _PolynomeBasis, SplineOrder = _SplineOrder };
  typedef path::Spline<PolynomeBasis, SplineOrder> Spline;
  typedef typename Spline::Ptr_t SplinePtr_t;
  typedef std::vector<SplinePtr_t> Splines_t;

  /// \name Spline convinience functions
  /// \{

  /// Copy a vector of Spline
  static void copy(const Splines_t& in, Splines_t& out);

  /// Sets the parameters each spline.
  /// \todo make this function static (currently, it only needs the
  ///       robot number dof.
  void updateSplines(Splines_t& spline, const vector_t& param) const;

  /// Gets the parameters each spline.
  /// \todo make this function static (currently, it only needs the
  ///       robot number dof.
  void updateParameters(vector_t& param, const Splines_t& spline) const;

  /// Returns res = (1 - alpha) * a + alpha * b
  static void interpolate(const Splines_t& a, const Splines_t& b,
                          const value_type& alpha, Splines_t& res);

  /// \}

 protected:
  SplineGradientBasedAbstract(const ProblemConstPtr_t& problem);

  /// \name Spline creation
  /// \{

  /// Flatten path and remove path of zero length.
  static PathVectorPtr_t cleanInput(const PathVectorPtr_t& input);

  /// Spline steering method.
  typedef steeringMethod::Spline<PolynomeBasis, SplineOrder> SSM_t;
  typename SSM_t::Ptr_t steeringMethod_;

  /// Convert a straight path into a spline and append to vector
  ///
  /// \param path straight path,
  /// \retval splines vector of splines
  ///
  /// Build a spline starting from straight path initial configuration
  /// and ending at straight path final configuration.
  /// derivatives (up to an order depending on the spline degree) at
  /// start and end are set to zero.
  void appendEquivalentSpline(const StraightPathPtr_t& path,
                              Splines_t& splines) const;

  /// Convert an interpolated path into a spline and append to vector
  ///
  /// \param path interpolated path,
  /// \retval splines vector of splines
  ///
  /// Build a spline starting from interpolated path initial
  /// configuration and ending at interpolated path final
  /// configuration.  derivatives (up to an order depending on
  /// the spline degree) at start and end are set to zero.  If
  /// the interpolated path as only two waypoint, treat it as
  /// a straight path. Otherwise, throw an exception.
  void appendEquivalentSpline(const InterpolatedPathPtr_t& path,
                              Splines_t& splines) const;

  /// For each subpath of path, cast it into a known path and calls
  /// appropriate appendEquivalentSpline.
  /// \param splines the output will be pushed back into this vector.
  void appendEquivalentSpline(const PathVectorPtr_t& path,
                              Splines_t& splines) const;

  /// \}

  /// \name Path validation
  /// \{

  /// Path validation
  /// Its size is the number of spline paths.
  std::vector<PathValidationPtr_t> validations_;

  /// Initialize validations_.
  /// Store a pointer to the path validation of the problem for
  /// each spline.
  virtual void initializePathValidation(const Splines_t& splines);

  typedef std::vector<std::pair<PathValidationReportPtr_t, std::size_t> >
      Reports_t;
  /// Calls each validations_ on the corresponding spline.
  /// \param reordering order in which the path validation is run.
  ///                   It is assumed that reordering is a permutation
  ///                   of [0, splines.size()[.
  /// \param stopAtFirst if true, then return only at most one report
  ///                    corresponding to the first encountered collision.
  /// \param reorder Put the portion in collision first in order to
  ///                improve performance of next collision check.
  Reports_t validatePath(const Splines_t& splines,
                         std::vector<std::size_t>& reordering, bool stopAtFirst,
                         bool reorder) const;

  /// \}

  /// \name Constraint creation
  /// \{

  typedef Eigen::RowBlockIndices RowBlockIndices;
  typedef std::vector<bool> Bools_t;
  typedef std::vector<size_type> Indices_t;
  struct SplineOptimizationData {
    SplineOptimizationData() {}
    SplineOptimizationData(size_type rDof) { activeParameters.addRow(0, rDof); }

    /// The set of constraint of the corresponding path.
    ConstraintSetPtr_t set;
    /// A copy of the explicit solver included in \ref set
    shared_ptr<constraints::ExplicitConstraintSet> es;

    /// Variable on which we can optimize.
    /// Other variables are fully constrained.
    RowBlockIndices activeParameters;
  };
  typedef std::vector<SplineOptimizationData> SplineOptimizationDatas_t;

  void jointBoundConstraint(const Splines_t& splines,
                            LinearConstraint& lc) const;

  /// Unused
  std::size_t addBoundConstraints(const Indices_t& bci,
                                  const LinearConstraint& bc,
                                  Bools_t& activeConstraint,
                                  LinearConstraint& constraint) const;

  /// Mostly for debugging purpose
  Indices_t validateBounds(const Splines_t& splines,
                           const LinearConstraint& lc) const;

  /// Add the linear constraint to connect consecutive splines together.
  void addContinuityConstraints(const Splines_t& splines,
                                const size_type maxOrder,
                                const SplineOptimizationDatas_t& ess,
                                LinearConstraint& continuity);

  /// \}

  PathVectorPtr_t buildPathVector(const Splines_t& splines) const;

  DevicePtr_t robot_;

 private:
  SplinePtr_t steer(ConfigurationIn_t q0, ConfigurationIn_t q1,
                    value_type length) const;

  /// Maybe
  // void addCollisionConstraint (const std::size_t idxSpline,
  // const SplinePtr_t& spline, const SplinePtr_t& nextSpline,
  // const SolverPtr_t& solver,
  // const CollisionPathValidationReportPtr_t& report,
  // LinearConstraint& collision, CollisionFunctions& functions) const;
};  // SplineGradientBasedAbstract
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_ABSTRACT_HH
