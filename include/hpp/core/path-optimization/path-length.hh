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

#ifndef HPP_CORE_PATH_OPTIMIZATION_PATH_LENGTH_HH
# define HPP_CORE_PATH_OPTIMIZATION_PATH_LENGTH_HH

# include <hpp/core/path-optimization/cost.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// Path length as a cost for optimization
      ///
      /// Instances are initialized by
      /// \li way points stored in a PathVector,
      /// \li a WeighedDistance object that defines the length of the
      ///     interpolations between way points,
      /// \li weight values to compute the weighted cost.
      ///
      /// The input of the function is a vector concatenating the way points
      /// of the path, excluding the first and last way points.
      class HPP_CORE_DLLAPI PathLength : public Cost
      {
      public:
	static PathLengthPtr_t create (const WeighedDistancePtr_t& distance,
				       const PathVectorPtr_t& path);
      protected:
	PathLength (const WeighedDistancePtr_t& distance,
		    const PathVectorPtr_t& path);
	virtual void impl_compute (vectorOut_t result,
				   vectorIn_t argument) const;

	virtual void impl_jacobian (matrixOut_t jacobian,
				    vectorIn_t arg) const;

	/// Return an approximation of the Hessian at minimum
	/// \retval hessian Hessian matrix of right size
	virtual void hessian (matrixOut_t result) const;
      private:
	/// Weight are computed according to initial partial
	/// paths lengths with regard to the total length.
	void computeLambda (const PathVectorPtr_t& path) const;

	std::size_t nbPaths_;
	WeighedDistancePtr_t distance_;
	Configuration_t initial_;
	Configuration_t end_;
	DevicePtr_t robot_;
	size_type configSize_;
	size_type numberDofs_;
	mutable vector_t lambda_;
      }; // PathLength
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_PATH_LENGTH_HH
