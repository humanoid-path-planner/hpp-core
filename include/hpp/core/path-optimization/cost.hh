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

#ifndef HPP_CORE_PATH_OPTIMIZATION_COST_HH
# define HPP_CORE_PATH_OPTIMIZATION_COST_HH

# include <hpp/constraints/differentiable-function.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// numerical cost for path optimization
      ///
      /// Provides an initial guess for the Hessian to initialize quasi-Newton
      /// methods.
      class HPP_CORE_DLLAPI Cost : public DifferentiableFunction
      {
      public:
	/// Return an approximation of the Hessian at minimum
	/// \retval hessian Hessian matrix of right size
	virtual void hessian (matrixOut_t hessian) const = 0;

      protected:
	Cost (size_type inputSize, size_type inputDerivativeSize,
	      const std::string& name) :
	  DifferentiableFunction (inputSize, inputDerivativeSize,
                                  LiegroupSpace::R1 (), name)
	{
	}
      }; // Cost
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_COST_HH
