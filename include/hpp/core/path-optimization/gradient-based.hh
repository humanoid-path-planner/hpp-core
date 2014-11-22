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

#ifndef HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
# define HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH

# include <hpp/core/path-optimizer.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      class HPP_CORE_DLLAPI GradientBased : public PathOptimizer
      {
      public:
	/// Return shared pointer to new object.
	/// Default cost is path length.
	static GradientBasedPtr_t create (const Problem& problem);
	/// Set cost
	void cost (const CostPtr_t& cost);
	/// Optimize path
	virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);
      protected:
	GradientBased (const Problem& problem);
      private:
	void pathToVector (const PathVectorPtr_t& path, vectorOut_t x) const;
	void vectorToPath (vectorIn_t x1, const PathVectorPtr_t& result) const;
	void integrate (vectorIn_t xPrev, vectorIn_t step, vectorOut_t x) const;
	mutable CostPtr_t cost_;
	DevicePtr_t robot_;
	size_type configSize_;
	size_type numberDofs_;
	mutable Configuration_t initial_;
	mutable Configuration_t end_;
	WeighedDistancePtr_t distance_;
	SteeringMethodStraightPtr_t steeringMethod_;
	// Identity matrix and estimation of Hessian inverse
	matrix_t I_, H_;
	// Jacobian of constraints
	matrix_t Jconstraints_;
	value_type alpha_;
      }; // GradientBased
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
