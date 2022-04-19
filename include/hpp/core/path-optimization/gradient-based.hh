//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
#define HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/steering-method/straight.hh>

namespace hpp {
namespace core {
/// \addtogroup path_optimization
/// \{
namespace pathOptimization {
class CollisionConstraintsResult;
typedef std::vector<CollisionConstraintsResult> CollisionConstraintsResults_t;

class HPP_CORE_DLLAPI GradientBased : public PathOptimizer {
 public:
  /// Return shared pointer to new object.
  /// Default cost is path length.
  static GradientBasedPtr_t create(const ProblemConstPtr_t& problem);

  /// Optimize path
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

 protected:
  GradientBased(const ProblemConstPtr_t& problem);

 private:
  typedef Eigen::JacobiSVD<matrix_t> Jacobi_t;

  void compressHessian(matrixIn_t normal, matrixOut_t small) const;
  void compressVector(vectorIn_t normal, vectorOut_t small) const;
  void uncompressVector(vectorIn_t small, vectorOut_t normal) const;

  void initialize(const PathVectorPtr_t& path);

  /// Convert a path into a vector
  void pathToVector(const PathVectorPtr_t& path, vectorOut_t x) const {
    size_type index = 0;
    for (std::size_t i = 0; i < path->numberPaths() - 1; ++i) {
      const PathPtr_t& localPath = path->pathAtRank(i);
      value_type t1 = localPath->timeRange().second;
      (*localPath)(x.segment(index, configSize_), t1);
      index += configSize_;
    }
    assert(index == x.size());
  }

  /// Convert a vector into a path
  void vectorToPath(vectorIn_t x, const PathVectorPtr_t& result) const {
    Configuration_t q0 = initial_, q1;
    size_type index = 0;
    while (index < x.size()) {
      q1 = x.segment(index, configSize_);
      PathPtr_t p = (*steeringMethod_)(q0, q1);
      result->appendPath(p);
      q0 = q1;
      index += configSize_;
    }
    q1 = end_;
    PathPtr_t p = (*steeringMethod_)(q0, q1);
    result->appendPath(p);
  }

  void initializeProblemConstraints();
  /// Compute value and Jacobian of the problem constraints
  ///
  /// \param x input path as a vector
  ///
  /// Constraints of the problem are applied to each waypoint. Therefore,
  /// the first 6n lines of the Jacobian and of the value are computed,
  /// where n is the number of waypoints.
  void updateProblemConstraints(vectorIn_t x);

  /// Check whether constraints are satisfied
  ///
  /// \param x input path as a vector,
  /// \param path input path (same path as x)
  /// \param collisionConstraints vector of collision constraints
  ///
  /// Check that
  ///   \li each waypoint satisfies the problem constraints, and
  ///   \li each collision constraint drift is within bounds that ensures
  ///       the user that there is no collision
  ///
  bool constraintsSatisfied(
      vectorOut_t& x, PathVectorPtr_t& path,
      CollisionConstraintsResults_t& collisionConstraints);

  /// Solve problem constraints and collision constraints
  ///
  /// \retval x path that satisfies the constraints as a vector,
  /// \retval path path that satisfies the constraints (same as x)
  /// \param collisionConstraints vector of collision constraints
  ///
  /// While constraints are not satisfied
  /// (method GradientBased::constraintsSatisfied), linearize constraints
  /// (problem and collision) around current path and solve linearized
  /// constraints regardless of the cost
  bool solveConstraints(vectorOut_t x, PathVectorPtr_t& path,
                        CollisionConstraintsResults_t& collisionConstraints);
  /// Process path integration
  void integrate(vectorIn_t x0, vectorIn_t step, vectorOut_t x1) const;

  /// Compute iteration of optimization program
  vector_t computeIterate(vectorIn_t x) const;

  /// Display path waypoints in log file
  void displayPath(vectorIn_t x, std::string
#ifdef HPP_DEBUG
                                     prefix
#endif
  ) const {
    hppDout(info, prefix);
    for (int i = 0; i < x.size(); i++) hppDout(info, x[i]);
    hppDout(info, "finish path parsing");
  }

  /// Add a collision constraint
  ///
  /// \param ccr contains parameter of collision configuration
  /// \param latest valid path
  /// Add lines to the Jacobian and fill with values corresponding to
  /// new relative position constraint.
  /// Resize right hand side.
  void addCollisionConstraint(const CollisionConstraintsResult& ccr,
                              const PathVectorPtr_t& path) const;

  /// Update right hand side of constraints
  ///
  /// \param list of collision constraints
  /// \param latest valid path
  ///
  /// Update right hand side with current path so that latest valid path
  /// perfectly satisfies constraints.
  void updateRightHandSide(
      const CollisionConstraintsResults_t& collisionConstraints,
      const PathVectorPtr_t& path) const;

  /// Get constraints of problem and update local jacobian J_
  bool getProblemConstraints();

  mutable CostPtr_t cost_;
  DevicePtr_t robot_;
  size_type configSize_;
  size_type robotNumberDofs_;
  size_type robotNbNonLockedDofs_;
  size_type numberDofs_;
  size_type fSize_;  // dimension of collision constraints
  mutable Configuration_t initial_;
  mutable Configuration_t end_;
  WeighedDistancePtr_t distance_;
  steeringMethod::StraightPtr_t steeringMethod_;
  /// I_, H_ Identity matrix and cost Hessian respectively
  /// Jf_  collision-constraints jacobian for now
  /// J_ problem-constraints jacobian
  /// Aplusb = Jf^{+}*b (for affine constraints)
  mutable matrix_t H_, Hinverse_, J_;
  mutable matrix_t Hz_, gz_;
  mutable vector_t stepNormal_;
  mutable bool fullRank_;
  mutable size_type nbWaypoints_;
  mutable value_type alpha_;
  mutable rowvector_t rgrad_;
  mutable vector_t rhs_;
  mutable vector_t value_;
  mutable vector_t p_, p0_;
  value_type epsilon_;
  std::size_t iterMax_;
  value_type alphaInit_;  // .1, .2, .4
  value_type alphaMax_;
};  // GradientBased
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
