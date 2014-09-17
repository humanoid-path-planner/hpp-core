//
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
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

#ifndef HPP_CORE_S_PATH_OPTIMIZER_HH
# define HPP_CORE_S_PATH_OPTIMIZER_HH


# include <hpp/core/path-optimizer.hh>



namespace hpp {
  namespace core {
    
    /// Optimizer of splinePath
    ///
    /// Path optimizer that tries to optimize the values of a vector of control point in order to 
    /// minimize an associated  cost function //TODO ? Work on knots vector ??
    ///
    /// \note The optimizer assumes that the input path contains a splinePath
    class HPP_CORE_DLLAPI SPathOptimizer : public PathOptimizer
    {
    public:
    
    /// Create a CubicBSpline from PathVector on all doF
    CubicBSpline_t pvToCompleteSpline(const PathVectorPtr_t& pv) const;
    
    /// Create a CubicBSpline from PathVector on x,y DoF
    CubicBSpline_t pvToSmallSpline(const PathVectorPtr_t& pv) const;
    
    /// Create a PathVector of SplinePath (dimension of original path) and return shared pointer
    PathVectorPtr_t completeSplineToPv(const PathVectorPtr_t& original, const CubicBSpline_t& spline) const;
    
    /// Create a PathVector of SplinePath (reduced dimension) and return shared pointer
    PathVectorPtr_t smallSplineToPv(const PathVectorPtr_t& original, const CubicBSpline_t& spline) const;
    
      /// Return shared pointer to new object.
      static SPathOptimizerPtr_t create (const Problem& problem);

      /// Optimize path
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path) const;
    protected:
      SPathOptimizer (const Problem& problem);
    }; // class SPathOptimizer
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_S_PATH_OPTIMIZER_HH
