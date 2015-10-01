//
// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH
# define HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH

# include <hpp/core/path-optimizer.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// \addtogroup path_optimization
      /// \{

      /// Partial shortcut
      ///
      /// Path optimizer that iteratively samples random configurations along a
      /// path and that tries to connect these configurations by a call to
      /// the steering method.
      ///
      /// \note The optimizer assumes that the input path is a vector of optimal
      ///       paths for the distance function.
      class HPP_CORE_DLLAPI PartialShortcut : public PathOptimizer
      {
        public:
          /// Return shared pointer to new object.
          static PartialShortcutPtr_t create (const Problem& problem);

          /// Optimize path
          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

          struct Parameters {
            bool removeLockedJoints;
            std::size_t numberOfConsecutiveFailurePerJoints;
            value_type progressionMargin;
            Parameters ();
          } parameters;

        protected:
          PartialShortcut (const Problem& problem);

        private:
          PathVectorPtr_t generatePath (
              PathVectorPtr_t path, const JointPtr_t joint,
              const value_type t1, ConfigurationIn_t q1,
              const value_type t2, ConfigurationIn_t q2) const;

          JointVector_t generateJointVector (const PathVectorPtr_t& pv) const;

          /// try direct path on each joint in jvIn.
          /// \param jvIn contains the joints on which optimization should be
          ///        tried
          /// \param jvOut contains the joints of jvIn on which optimization
          ///        failed.
          /// \return the optimized path
          PathVectorPtr_t optimizeFullPath (const PathVectorPtr_t& pv,
              const JointVector_t& jvIn, JointVector_t& jvOut) const;

          /// optimize each joint in jvIn.
          /// \param jvIn contains the joints on which optimization should be
          ///        tried
          /// \return the optimized path
          PathVectorPtr_t optimizeRandom (const PathVectorPtr_t& pv,
              const JointVector_t& jv) const;
      }; // class RandomShortcut
      /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SHORTCUT_HH
