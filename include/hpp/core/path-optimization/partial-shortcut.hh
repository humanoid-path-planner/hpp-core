//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH
#define HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH

#include <hpp/core/path-optimizer.hh>

namespace hpp {
namespace core {
namespace pathOptimization {

typedef std::vector<JointConstPtr_t> JointStdVector_t;

/// \addtogroup path_optimization
/// \{

/// Partial shortcut
///
/// The algorithm has 3 steps:
/// \li find a suitable set of joints that can be optimized.
/// \li try a direct path for each of this joints. If this step fails for
///     a joint, then the joint is inserted in a input set of next step.
/// \li try to find random shortcut on each joint in the set.
///
/// See Parameters for information on how to tune the algorithm.
///
/// \note The optimizer assumes that the input path is a vector of optimal
///       paths for the distance function.
struct PartialShortcutTraits {
  static bool removeLockedJoints() { return true; }
  static bool onlyFullShortcut() { return false; }
  static std::size_t numberOfConsecutiveFailurePerJoints() { return 5; }
  static value_type progressionMargin() { return 1e-3; }
};

class HPP_CORE_DLLAPI PartialShortcut : public PathOptimizer {
 public:
  /// Return shared pointer to new object.
  template <typename Traits>
  static PartialShortcutPtr_t createWithTraits(
      const ProblemConstPtr_t& problem);

  /// Return shared pointer to new object.
  static PartialShortcutPtr_t create(const ProblemConstPtr_t& problem);

  /// Optimize path
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

  struct Parameters {
    /// Whether of not the joint that are locked by the constraints
    /// in the path should not be optimized.
    /// This is safe if you have the same constraints along the path.
    /// Defaults to true
    bool removeLockedJoints;

    /// Set it to true if you want to skip step 3
    /// Defaults to false
    bool onlyFullShortcut;

    /// The optimization will stop after a number of consecutive failure
    /// on each joint.
    /// Defaults to 5
    std::size_t numberOfConsecutiveFailurePerJoints;

    /// An iteration will be considered as a failure is the path length
    /// did not decrease more than progressionMargin.
    /// Defaults is 1e-3
    value_type progressionMargin;

    Parameters();
  } parameters;

 protected:
  PartialShortcut(const ProblemConstPtr_t& problem);

 private:
  PathVectorPtr_t generatePath(PathVectorPtr_t path, JointConstPtr_t joint,
                               const value_type t1, ConfigurationIn_t q1,
                               const value_type t2, ConfigurationIn_t q2) const;

  JointStdVector_t generateJointVector(const PathVectorPtr_t& pv) const;

  /// try direct path on each joint in jvIn.
  /// \param jvIn contains the joints on which optimization should be
  ///        tried
  /// \param jvOut contains the joints of jvIn on which optimization
  ///        failed.
  /// \return the optimized path
  PathVectorPtr_t optimizeFullPath(const PathVectorPtr_t& pv,
                                   const JointStdVector_t& jvIn,
                                   JointStdVector_t& jvOut) const;

  /// optimize each joint in jvIn.
  /// \param jvIn contains the joints on which optimization should be
  ///        tried
  /// \return the optimized path
  PathVectorPtr_t optimizeRandom(const PathVectorPtr_t& pv,
                                 const JointStdVector_t& jv) const;
};  // class RandomShortcut
/// \}

template <typename Traits>
PartialShortcutPtr_t PartialShortcut::createWithTraits(
    const ProblemConstPtr_t& problem) {
  PartialShortcut* ptr = new PartialShortcut(problem);
  ptr->parameters.removeLockedJoints = Traits::removeLockedJoints();
  ptr->parameters.onlyFullShortcut = Traits::onlyFullShortcut();
  ptr->parameters.progressionMargin = Traits::progressionMargin();
  ptr->parameters.numberOfConsecutiveFailurePerJoints =
      Traits::numberOfConsecutiveFailurePerJoints();
  return PartialShortcutPtr_t(ptr);
}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH
