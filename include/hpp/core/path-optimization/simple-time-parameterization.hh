// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH
# define HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH

# include <hpp/core/path-optimizer.hh>

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
       *  Let \f$ v \f$ be a velocity bound (Path::velocityBound) of \f$ q \f$ on \f$ [s_0, s_1] \f$.
       *  Then \f$ P'_n(t) \le B \f$ where
       *  \f[
       *   B = \min_i \frac{u_i}{v_i}, \frac{-l_i}{v_i}
       *  \f]
       * 
       *  The constraints on \f$ P_n \f$ are:
       *  - \f$ P_n(0) = s_0 \f$
       *  - \f$ P_n(T) = s_1 \f$
       *  - \f$ P'_n(t) \le B \f$
       *  - \f$ P'_n(0) = 0 \f$ (in the third order case only)
       *  - \f$ P'_n(T) = 0 \f$ (in the third order case only)
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
       */

      class HPP_CORE_DLLAPI SimpleTimeParameterization : public PathOptimizer
      {
        public:
          /// Return shared pointer to new object.
          static SimpleTimeParameterizationPtr_t create (const Problem& problem);

          /// Optimize path
          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        protected:
          SimpleTimeParameterization (const Problem& problem);
      }; // class SimpleTimeParameterization
      /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_SIMPLE_TIME_PARAMETERIZATION_HH
