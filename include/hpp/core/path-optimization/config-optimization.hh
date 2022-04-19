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

#ifndef HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
#define HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH

#include <functional>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
/// \addtogroup path_optimization
/// \{

/// Optimize the waypoints of the path and optionally add the
/// constraint::ConfigurationConstraint to the ConfigProjector of the
/// path.
///
/// See Parameters for information on how to tune the algorithm.
///
/// \note The optimizer assumes that the input path is a vector of optimal
///       paths for the distance function.
struct ConfigOptimizationTraits {
  static bool addConfigConstraintToPath() { return false; }

  static std::size_t numberOfPass() { return 5; }

  static std::size_t numberOfIterations() { return 3; }

  static value_type alphaInit() { return 0.4; }

  static Configuration_t getGoal(const PathVector& path) {
    return path.initial();
  }

  static ConfigProjectorPtr_t getConfigProjector(const PathPtr_t& before,
                                                 const PathPtr_t& after,
                                                 bool& isReverse);

  static bool shouldFilter(JointConstPtr_t joint, const size_type iDof);
};

class HPP_CORE_DLLAPI ConfigOptimization : public PathOptimizer {
 public:
  /// Return shared pointer to new object.
  template <typename Traits>
  static ConfigOptimizationPtr_t createWithTraits(
      const ProblemConstPtr_t& problem);

  /// Return shared pointer to new object.
  static ConfigOptimizationPtr_t create(const ProblemConstPtr_t& problem);

  /// Optimize path
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

  struct Parameters {
    /// Defaults to false
    bool addConfigConstraintToPath;

    std::size_t numberOfPass;

    std::size_t numberOfIterations;

    value_type alphaInit;

    std::function<Configuration_t(const PathVector&)> getGoal;

    std::function<ConfigProjectorPtr_t(const PathPtr_t&, const PathPtr_t&,
                                       bool&)>
        getConfigProjector;

    std::function<bool(JointConstPtr_t, const size_type)> shouldFilter;

    Parameters();
  } parameters;

 protected:
  ConfigOptimization(const ProblemConstPtr_t& problem);

  virtual constraints::ImplicitPtr_t createNumConstraint(
      const PathVector& path) const;

  struct Optimizer {
    ConfigProjectorPtr_t proj;
    virtual bool optimize(ConfigurationOut_t q,
                          const std::size_t numIter) const;
  };

  typedef std::vector<Optimizer> Optimizers_t;

  virtual std::size_t buildOptimizers(const PathVector& pv,
                                      Optimizers_t& projectors);

 private:
  bool isValid(const PathPtr_t& p) const;

  void buildConfigVector(const PathVector& path, vectorOut_t configs) const;

  template <bool forward>
  bool pass(vectorIn_t configs, vectorOut_t newConfigs,
            const Optimizers_t& optimizers, const std::size_t& index,
            PathVectorPtr_t opted, bool& didChange) const;
};  // class RandomShortcut
/// \}

template <typename Traits>
ConfigOptimizationPtr_t ConfigOptimization::createWithTraits(
    const ProblemConstPtr_t& problem) {
  ConfigOptimization* ptr = new ConfigOptimization(problem);
  ptr->parameters.addConfigConstraintToPath =
      Traits::addConfigConstraintToPath();
  ptr->parameters.numberOfPass = Traits::numberOfPass();
  ptr->parameters.numberOfIterations = Traits::numberOfIterations();
  ptr->parameters.alphaInit = Traits::alphaInit();
  ptr->parameters.getGoal = Traits::getGoal;
  ptr->parameters.getConfigProjector = Traits::getConfigProjector;
  ptr->parameters.shouldFilter = Traits::shouldFilter;
  return ConfigOptimizationPtr_t(ptr);
}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
