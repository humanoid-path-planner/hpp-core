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

# include <boost/function.hpp>

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

        static value_type alphaInit () { return 0.4; }

        static Configuration_t getGoal (const PathVector& path)
        { return path.initial (); }

        static ConfigProjectorPtr_t getConfigProjector
          (const PathPtr_t& before, const PathPtr_t& after, bool& isReverse);

        static bool shouldFilter (const JointPtr_t joint, const size_type iDof);
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

            value_type alphaInit;

            boost::function <Configuration_t (const PathVector&)> getGoal;

            boost::function <
              ConfigProjectorPtr_t (const PathPtr_t&, const PathPtr_t&, bool&)
              > getConfigProjector;

            boost::function <bool (const JointPtr_t, const size_type)>
              shouldFilter;

            Parameters ();
          } parameters;

        protected:
          ConfigOptimization (const Problem& problem);

          virtual NumericalConstraintPtr_t createNumConstraint
            (const PathVector& path) const;

          struct Optimizer {
            ConfigProjectorPtr_t proj;
            virtual bool optimize (ConfigurationOut_t q,
                const std::size_t numIter,
                const value_type alpha) const;
          };

          typedef std::vector <Optimizer> Optimizers_t;

          virtual std::size_t buildOptimizers (const PathVector& pv,
              Optimizers_t& projectors);

        private:
          bool isValid (const PathPtr_t& p) const;

          void buildConfigVector (const PathVector& path,
              vectorOut_t configs) const;

          template <bool forward> bool pass (
              vectorIn_t configs, vectorOut_t newConfigs,
              const Optimizers_t& optimizers, const std::size_t& index,
              const value_type& alpha, PathVectorPtr_t opted, bool& didChange)
            const;
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
        ptr->parameters.alphaInit = Traits::alphaInit ();
        ptr->parameters.getGoal = Traits::getGoal;
        ptr->parameters.getConfigProjector = Traits::getConfigProjector;
        ptr->parameters.shouldFilter = Traits::shouldFilter;
        return ConfigOptimizationPtr_t (ptr);
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
