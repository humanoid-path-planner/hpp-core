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

      /// Change the time of straight paths so that the velocity does not
      /// exceeds the velocity limits.
      ///
      /// Parameter SimpleTimeParameterization/Safety (value_type) defines
      /// is used to rescale the velocity limit.

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
