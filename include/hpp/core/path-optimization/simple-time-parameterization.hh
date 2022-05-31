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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH
#define HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH

#include <hpp/core/path-optimizer.hh>

namespace hpp {
namespace core {
namespace pathOptimization {
/// \addtogroup path_optimization
/// \{

/** Add a TimeParameterization to paths so that the velocity does not
 *  exceeds the velocity limits.
 *
 *  Parameter SimpleTimeParameterization/safety (value_type) defines
 *  is used to rescale the velocity limit.
 *
 *  Parameter SimpleTimeParameterization/velocity (bool) defines
 *  whether to use first or third order Polynomial.
 *
 *  Let the initial path be
 *  \f[ \begin{align*}
 *    q \colon  [s_0, s_1] & \to \mathcal{C} \\
 *               s         & \mapsto q(s)
 *  \end{align*} \f]
 *  We want to find a increasing Polynomial
 *  \f[ \begin{align*}
 *    P_n \colon  [0, T] & \to [s_0, s_1] \\
 *                 t     & \mapsto s
 *  \end{align*} \f]
 *  where \f$n\f$ is the polynom degree.
 *  The time parameterized path should satisfy the velocity limits so:
 *  \f[
 *    l_i \le \dot q_i (P_n(t)) = P'_n(t) \frac{dq_i }{ds} (P_n(t)) \le u_i \\
 *    P'_n(t) | \frac{dq_i }{ds} (P_n(t)) | \le \min u_i, -l_i \\
 *  \f]
 *  where \f$ u_i > 0 \f$ and \f$ l_i < 0 \f$ are the velocity limits.
 *
 *  Let \f$ v \f$ be a velocity bound (Path::velocityBound) of \f$ q \f$ on \f$
 * [s_0, s_1] \f$. Then \f$ P'_n(t) \le B \f$ where \f[ B = \min_i
 * \frac{u_i}{v_i}, \frac{-l_i}{v_i} \f]
 *
 *  The constraints on \f$ P_n \f$ are:
 *  - \f$ P_n(0) = s_0 \f$
 *  - \f$ P_n(T) = s_1 \f$
 *  - \f$ P^{(1)}_n(t) \le B \f$
 *  - \f$ P^{(1)}_n(0) = 0 \f$ (from the third order case only)
 *  - \f$ P^{(1)}_n(T) = 0 \f$ (from the third order case only)
 *  - \f$ P^{(2)}_n(0) = 0 \f$ (from the fifth order case only)
 *  - \f$ P^{(2)}_n(T) = 0 \f$ (from the fifth order case only)
 *  - \f$ |P^{(2)}_n(t)| \le C \f$ (from the fifth order case only)
 *
 *  The solutions are:
 *  \li First order case:
 *    \f[ \begin{align*}
 *    a_0 &= s_0              \\
 *    a_1 &= B                \\
 *    T &= \frac{s_1 - s_0}{B} \\
 *    \end{align*} \f]
 *
 *  \li Third order case:
 *    \f[ \begin{align*}
 *    T &= 3 \frac{s_1 - s_0}{2 B} \\
 *    a_0 &= s_0              \\
 *    a_1 &= 0                \\
 *    a_2 &= 3 \frac{s_1 - s_0}{T^2} \\
 *    a_3 &= - \frac{2 a_2}{3 T} \\
 *    \end{align*} \f]
 *    In this case, \f$ P^{(1)}_3(t) = 6 \frac{s_1-s_0}{T} \frac{t}{T}
 *    \left(1 - \frac{t}{T}\right) \ge 0 \f$.
 *
 *  \li Fifth order case:\n
 *    Let \f$ P_5(t) = \sum_i a_i t^i \f$.
 *    Trivially, \f$ a_0 = s_0 \f$, \f$ a_1 = 0 \f$ and \f$ a_2 = 0 \f$. \n
 *    Then, \f$ P_5(T) = s_1 \f$, \f$ P^{(1)}_5(T) = 0 \f$ and
 *    \f$ P^{(2)}_5(T) = 0 \f$ implies:
 *    \f[
 *    \left(\begin{array}{ccc}
 *       T^3 &    T^4 &    T^5 \\
 *     3 T^2 &  4 T^3 &  5 T^4 \\
 *     6 T   & 12 T^2 & 20 T^3 \\
 *    \end{array}\right)
 *    \times
 *    \left(\begin{array}{c}
 *     a_3 \\
 *     a_4 \\
 *     a_5 \\
 *    \end{array}\right)
 *    =
 *    \left(\begin{array}{c}
 *     s_1 - s_0 \\
 *     0         \\
 *     0         \\
 *    \end{array}\right)
 *    \f]
 *    \f[
 *    \left(\begin{array}{c}
 *     a_3 \\
 *     a_4 \\
 *     a_5 \\
 *    \end{array}\right)
 *    =
 *    \frac{s_1-s_0}{\det(M)}
 *    \times
 *    \left(\begin{array}{c}
 *     80 T^6 - 60 T^6 \\
 *     30 T^5 - 60 T^5 \\
 *     36 T^4 - 24 T^4 \\
 *    \end{array}\right)
 *    \f]
 *    And thus:
 *    \f[
 *    \left(\begin{array}{c}
 *     a_3 \\
 *     a_4 \\
 *     a_5 \\
 *    \end{array}\right)
 *    =
 *    \frac{s_1-s_0}{T^5}
 *    \times
 *    \left(\begin{array}{c}
 *     10 T^2 \\
 *     -15  T \\
 *       6    \\
 *    \end{array}\right)
 *    \f]
 *    Then \f$ P_5^{(1)}(t) = 30 \frac{s_1-s_0}{T}
 *    \left(\frac{t}{T}\right)^2
 *    \left(1 - \frac{t}{T}\right)^2 \ge 0 \f$ and
 *    \f$ P_5^{(2)}(t) = 60 \frac{s_1-s_0}{T^2}
 *    \left(\frac{t}{T}\right)
 *    \left(1 - \frac{t}{T}\right)
 *    \left(1 - 2\frac{t}{T}\right) \f$.
 *    We have \f$ \max{P_5^{(1)}(t)} = P_5^{(1)}(\frac{T}{2}) = \frac{30 (s_1 -
 * s_0)}{16 T} \f$. Let us compute \f$ \max{P_5^{(2)}(t)} \f$.
 *
 *    Let \f$ q(u) = 2u (u+\frac{1}{2})(u-\frac{1}{2})\f$.
 *    Then \f$ P_5^{(2)}(t) = 60 \frac{s_1-s_0}{T^2} q(\frac{t}{T}-\frac{1}{2})
 * \f$. For \f$ u \in [0,\frac{1}{2}]\f$, we have \f$ q(-u) = -q(u) \f$ and \f$
 * |q(u)| = -q(u) \le -q(\frac{1}{2\sqrt{3}}) = \frac{1}{6\sqrt{3}}\f$. So \f[
 * \max{P_5^{(2)}(t)} = \frac{10}{\sqrt{3}} \frac{s_1-s_0}{T^2} \f]
 *
 *    So
 *    \f[ \begin{align*}
 *    \frac{15 (s_1 - s_0)}{8 T}             &\le B \\
 *    \frac{10}{\sqrt{3}} \frac{s_1-s_0}{T^2} &\le C \\
 *    \end{align*} \f]
 *
 *    \f[ T = \max{
 *      \left\{
 *        \frac{15 (s_1 - s_0)}{8 B},
 *        \sqrt{\frac{10 (s_1 - s_0)}{\sqrt{3} B}}
 *      \right\}
 *      } \f]
 */

class HPP_CORE_DLLAPI SimpleTimeParameterization : public PathOptimizer {
 public:
  /// Return shared pointer to new object.
  static SimpleTimeParameterizationPtr_t create(
      const ProblemConstPtr_t& problem);

  /// Optimize path
  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

 protected:
  SimpleTimeParameterization(const ProblemConstPtr_t& problem);
};  // class SimpleTimeParameterization
/// \}
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH
