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

#ifndef HPP_CORE_PATH_OPTIMIZER_HH
#define HPP_CORE_PATH_OPTIMIZER_HH

#include <boost/date_time/posix_time/ptime.hpp>
#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
namespace core {
/// \addtogroup path_optimization
/// \{

/// Abstraction of path optimizer
///
class HPP_CORE_DLLAPI PathOptimizer {
 public:
  virtual ~PathOptimizer() {};

  /// Get problem
  ProblemConstPtr_t problem() const { return problem_; }

  /// Optimize path
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path) = 0;

  /// Interrupt path optimization
  void interrupt() { interrupt_ = true; }
  /// Set maximal number of iterations
  void maxIterations(const unsigned long int& n);
  /// set time out (in seconds)
  void timeOut(const double& timeOut);

 protected:
  /// Whether to interrupt computation
  /// Set to false at start of optimize method, set to true by method
  /// interrupt.
  bool interrupt_;

  PathOptimizer(const ProblemConstPtr_t& problem);

  PathPtr_t steer(ConfigurationIn_t q1, ConfigurationIn_t q2) const;

  void monitorExecution();

  void endIteration() { ++monitor_.iteration; }

  bool shouldStop() const;

  void initFromParameters();

 private:
  ProblemConstPtr_t problem_;

  /// Maximal number of iterations to solve a problem
  /// reaching this bound raises an exception.
  size_type maxIterations_;
  /// Time out (in seconds) before interrupting the planning
  double timeOut_;

  /// Information used to monitor the execution of the algorithm.
  /// This information is:
  /// \li initialized by \ref monitorExecution
  /// \li updated by \ref endIteration
  /// \li read by \ref shouldStop
  struct {
    bool enabled;
    size_type iteration;
    boost::posix_time::ptime timeStart;
  } monitor_;
};  // class PathOptimizer;
/// }
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZER_HH
