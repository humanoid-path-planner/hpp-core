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

#ifndef HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
# define HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH

# include <hpp/core/path-optimizer.hh>
# include <hpp/core/path-vector.hh>

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
        static bool addConfigConstraintToPath () { return false; }

        static std::size_t numberOfPass () { return 5; }

        static std::size_t numberOfIterations () { return 3; }

        static Configuration_t getGoal (const PathVector& path)
        { return path.initial (); }
      };

      class HPP_CORE_DLLAPI ConfigOptimization : public PathOptimizer
      {
        public:
          /// Return shared pointer to new object.
          template < typename Traits > static
            ConfigOptimizationPtr_t createWithTraits (const Problem& problem);

          /// Return shared pointer to new object.
          static ConfigOptimizationPtr_t create (const Problem& problem);

          /// Optimize path
          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

          struct Parameters {
            /// Defaults to false
            bool addConfigConstraintToPath;

            std::size_t numberOfPass;

            std::size_t numberOfIterations;

            boost::function <Configuration_t (const PathVector& path)> getGoal;

            Parameters ();
          } parameters;

        protected:
          ConfigOptimization (const Problem& problem);

        private:
          bool isValid (const PathPtr_t& p) const;

          NumericalConstraintPtr_t createNumConstraint
            (const PathVectorPtr_t& path) const;
      }; // class RandomShortcut
      /// \}

      template < typename Traits > ConfigOptimizationPtr_t
        ConfigOptimization::createWithTraits (const Problem& problem)
      {
        ConfigOptimization* ptr = new ConfigOptimization (problem);
        ptr->parameters.addConfigConstraintToPath =
          Traits::addConfigConstraintToPath ();
        ptr->parameters.numberOfPass = Traits::numberOfPass ();
        ptr->parameters.numberOfIterations = Traits::numberOfIterations ();
          Traits::addConfigConstraintToPath ();
        ptr->parameters.getGoal = Traits::getGoal;
        return ConfigOptimizationPtr_t (ptr);
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
