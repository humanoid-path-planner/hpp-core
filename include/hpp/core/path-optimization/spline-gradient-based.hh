// Copyright (c) 2017 CNRS
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH
# define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH

#include <hpp/constraints/explicit-solver.hh>

#include <hpp/constraints/hybrid-solver.hh>

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path/spline.hh>

#include <hpp/core/steering-method/spline.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_optimization
    /// \{
    namespace pathOptimization {
      template <int _PolynomeBasis, int _SplineOrder>
      class HPP_CORE_DLLAPI SplineGradientBased : public PathOptimizer
      {
        public:
          enum {
            PolynomeBasis = _PolynomeBasis,
            SplineOrder = _SplineOrder
          };
          typedef boost::shared_ptr<SplineGradientBased> Ptr_t;

          typedef path::Spline<PolynomeBasis, SplineOrder> Spline;
          typedef typename Spline::Ptr_t SplinePtr_t;
          typedef std::vector<SplinePtr_t> Splines_t;

          /// Return shared pointer to new object.
          /// Default cost is path length.
          static Ptr_t create (const Problem& problem);

          /// Optimize path
          /// 1* - Transform straight paths into splines
          /// 2* - Add continuity constraints
          /// 3  - Add problem constraints
          /// 4* - Make cost function
          /// 5* - Compute explicit representation of linear constraints.
          /// 6* :
          ///    1 - Compute cost hessian
          ///    2 - Compute optimum
          ///    3 - Check path for collision. If no collision, return solution
          ///    4 - add collision constraint
          ///    5 - re-compute explicit representation of linear constraints.
          /// 7* - Build result path.
          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        protected:
          typedef steeringMethod::Spline<PolynomeBasis, SplineOrder> SM_t;

          SplineGradientBased (const Problem& problem);

          // Path validation
          std::vector<PathValidationPtr_t> validations_;

          virtual void initializePathValidation(const Splines_t& splines);

          // Constraint creation

          struct LinearConstraint;
          typedef constraints::ExplicitSolver Solver_t;
          typedef Eigen::RowBlockIndexes RowBlockIndexes;
          struct SolverPtr_t {
            SolverPtr_t () {}
            SolverPtr_t (size_type rDof) { av.addRow (0, rDof); }

            ConstraintSetPtr_t set;
            boost::shared_ptr<Solver_t> es;

            /// Variable on which we can optimize.
            /// Other variables are fully constrained.
            RowBlockIndexes av;
          };
          typedef std::vector <SolverPtr_t> Solvers_t;

          virtual void addProblemConstraints (const PathVectorPtr_t& init, const Splines_t& splines, LinearConstraint& lc, Solvers_t& ess) const;

          void addProblemConstraintOnPath (const PathPtr_t& path, const size_type& idxSpline, const SplinePtr_t& spline, LinearConstraint& lc, SolverPtr_t& es) const;

          /// \param guessThr Threshold used to check whether the Jacobian
          ///                 contains rows of zeros, in which case the
          ///                 corresponding DoF is considered passive.
          Eigen::RowBlockIndexes computeActiveParameters (const PathPtr_t& path,
              const constraints::HybridSolver& hs,
              const value_type& guessThr = -1,
              const bool& useExplicitInput = false) const;

          DevicePtr_t robot_;
          bool checkOptimum_;

        private:
          struct ContinuityConstraint;
          struct QuadraticProblem;
          typedef std::vector <std::pair <CollisionPathValidationReportPtr_t,
                  std::size_t> > Reports_t;
          typedef std::vector <bool> Bools_t;
          typedef std::vector <size_type> Indexes_t;
          struct CollisionFunctions;

          void appendEquivalentSpline (const StraightPathPtr_t& path, Splines_t& splines) const;

          void appendEquivalentSpline (const InterpolatedPathPtr_t& path, Splines_t& splines) const;

          void appendEquivalentSpline (const PathVectorPtr_t& path, Splines_t& splines) const;

          void addContinuityConstraints (const Splines_t& splines, const size_type maxOrder, const Solvers_t& ess, LinearConstraint& continuity);

          Reports_t validatePath (const Splines_t& splines, bool stopAtFirst) const;

          Indexes_t validateBounds (const Splines_t& splines, const LinearConstraint& lc) const;

          std::size_t addBoundConstraints (const Indexes_t& bci, const LinearConstraint& bc,
              Bools_t& activeConstraint, LinearConstraint& constraint) const;

          void addCollisionConstraint (const std::size_t idxSpline,
              const SplinePtr_t& spline, const SplinePtr_t& nextSpline,
              const SolverPtr_t& solver,
              const CollisionPathValidationReportPtr_t& report,
              LinearConstraint& collision, CollisionFunctions& functions) const;

          bool findNewConstraint (LinearConstraint& constraint,
              LinearConstraint& collision, LinearConstraint& collisionReduced,
              CollisionFunctions& functions, const std::size_t iF,
              const SplinePtr_t& spline, const SolverPtr_t& solver) const;


          PathVectorPtr_t buildPathVector (const Splines_t& splines) const;

          void jointBoundConstraint (const Splines_t& splines, LinearConstraint& lc) const;

          bool isContinuous (const Splines_t& splines, const size_type maxOrder, const ContinuityConstraint& continuity) const;

          template <typename Cost_t> bool checkHessian (const Cost_t& cost, const matrix_t& H, const Splines_t& splines) const;

          /// \todo static
          void copy (const Splines_t& in, Splines_t& out) const;

          /// Returns res = (1 - alpha) * a + alpha * b
          void updateSplines (Splines_t& spline, const vector_t& param) const;

          /// Returns res = (1 - alpha) * a + alpha * b
          /// \todo static
          void interpolate (const Splines_t& a, const Splines_t& b,
              const value_type& alpha, Splines_t& res) const;

          void copyParam (const Splines_t& in, Splines_t& out) const;

          typename SM_t::Ptr_t steeringMethod_;

          // Continuity constraints
          // matrix_t Jcontinuity_;
          // vector_t rhsContinuity_;
      }; // GradientBased
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_GRADIENT_BASED_HH
