//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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
	  DifferentiableFunction (inputSize, inputDerivativeSize, 1, name)
	{
	}
      }; // Cost
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_COST_HH
