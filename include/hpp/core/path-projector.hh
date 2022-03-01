// Copyright (c) 2014, LAAS-CNRS
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
