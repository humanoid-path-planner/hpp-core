// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_CORE_PATHPROJECTOR_HH
# define HPP_CORE_PATHPROJECTOR_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// This class projects a path using constraints.
    class HPP_CORE_DLLAPI PathProjector
    {
      public:
        typedef hpp::core::Path Path;
        typedef hpp::core::PathPtr_t PathPtr_t;
        typedef hpp::core::PathVector PathVector;
        typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;

        /// Destructor
        virtual ~PathProjector ();

        /// Apply the constraints to the path.
        /// \param[in] the input path,
        /// \param[out] the output path.
        /// \return True if projection succeded
        bool apply (const PathPtr_t& path, PathPtr_t& projection) const;

      protected:
        /// Constructor
	///
	/// distance distance metric between configurations
	/// steeringMethod steering method used to build new paths. The instance
	///                is copied.
	/// copySteeringMethodConstraints whether to keep the constraints of the
	///                               input steering method.
        PathProjector (const DistancePtr_t& distance,
		       const SteeringMethodPtr_t& steeringMethod,
		       bool keepSteeringMethodConstraints = false);

        /// Method to be reimplemented by inherited class.
        virtual bool impl_apply (const PathPtr_t& path,
				 PathPtr_t& projection) const = 0;

        value_type d (ConfigurationIn_t q1, ConfigurationIn_t q2) const;
	PathPtr_t steer (ConfigurationIn_t q1, ConfigurationIn_t q2) const;
	SteeringMethodPtr_t steeringMethod_;
      private:
        DistancePtr_t distance_;
    };
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PATHPROJECTOR_HH
