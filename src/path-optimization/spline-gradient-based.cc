// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/constraints/svd.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/path-optimization/quadratic-program.hh>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/problem.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>
#include <path-optimization/spline-gradient-based/collision-constraint.hh>
#include <path-optimization/spline-gradient-based/cost.hh>

namespace hpp {
namespace core {
using constraints::ExplicitConstraintSet;
using constraints::solver::BySubstitution;
using pinocchio::Device;

namespace pathOptimization {
typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
    RowMajorMatrix_t;
typedef Eigen::Map<const vector_t> ConstVectorMap_t;
typedef Eigen::Map<vector_t> VectorMap_t;

typedef Eigen::BlockIndex BlockIndex;

HPP_DEFINE_TIMECOUNTER(SGB_findNewConstraint);

template <int NbRows>
VectorMap_t reshape(Eigen::Matrix<value_type, NbRows, Eigen::Dynamic,
                                  Eigen::RowMajor>& parameters) {
  return VectorMap_t(parameters.data(), parameters.size());
}

template <int NbRows>
ConstVectorMap_t reshape(const Eigen::Matrix<value_type, NbRows, Eigen::Dynamic,
                                             Eigen::RowMajor>& parameters) {
  return ConstVectorMap_t(parameters.data(), parameters.size());
}

template <int _PB, int _SO>
SplineGradientBased<_PB, _SO>::SplineGradientBased(
    const ProblemConstPtr_t& problem)
    : Base(problem), checkOptimum_(false) {}

// ----------- Convenience class -------------------------------------- //

/// TODO Two options:
/// - Split this class into two classes:
///   - Move generic part outside of this class
///   - Keep linearization
/// - Move all the code outside
template <int _PB, int _SO>
struct SplineGradientBased<_PB, _SO>::CollisionFunctions {
  void addConstraint(const typename CollisionFunction<SplinePtr_t>::Ptr_t& f,
                     const std::size_t& idx, const size_type& row,
                     const value_type& r) {
    assert(f->outputSize() == 1);
    functions.push_back(f);
    splineIds.push_back(idx);
    rows.push_back(row);
    ratios.push_back(r);
  }

  void removeLastConstraint(const std::size_t& n, LinearConstraint& lc) {
    assert(functions.size() >= n && std::size_t(lc.J.rows()) >= n);

    const std::size_t nSize = functions.size() - n;
    functions.resize(nSize);
    splineIds.resize(nSize);
    rows.resize(nSize);
    ratios.resize(nSize);

    lc.J.conservativeResize(lc.J.rows() - n, lc.J.cols());
    lc.b.conservativeResize(lc.b.rows() - n, lc.b.cols());
  }

  // Compute linearization
  // b = f(S(t))
  // J = Jf(S(p, t)) * dS/dp
  // f(S(t)) = b -> J * P = b
  void linearize(const SplinePtr_t& spline, const SplineOptimizationData& sod,
                 const std::size_t& fIdx, LinearConstraint& lc) {
    const typename CollisionFunction<SplinePtr_t>::Ptr_t& f = functions[fIdx];

    const size_type row = rows[fIdx], nbRows = 1,
                    rDof = f->inputDerivativeSize();
    const value_type t = spline->length() * ratios[fIdx];

    q.resize(f->inputSize());
    spline->eval(q, t);

    // Evaluate explicit functions
    if (sod.es) sod.es->solve(q);

    LiegroupElement v(f->outputSpace());
    f->value(v, q);

    J.resize(f->outputSize(), f->inputDerivativeSize());
    f->jacobian(J, q);

    // Apply chain rule if necessary
    if (sod.es) {
      Js.resize(sod.es->nv(), sod.es->nv());
      sod.es->jacobian(Js, q);

      sod.es->notOutDers().lview(J) =
          sod.es->notOutDers().lview(J).eval() +
          sod.es->outDers().transpose().rview(J).eval() *
              sod.es->jacobianNotOutToOut(Js).eval();
      sod.es->outDers().transpose().lview(J).setZero();
    }

    Spline::timeFreeBasisFunctionDerivative(0, ratios[fIdx],
                                            paramDerivativeCoeff);

    const size_type col = splineIds[fIdx] * Spline::NbCoeffs * rDof;
    for (size_type i = 0; i < Spline::NbCoeffs; ++i)
      lc.J.block(row, col + i * rDof, nbRows, rDof).noalias() =
          paramDerivativeCoeff(i) * J;

    lc.b.segment(row, nbRows) =
        lc.J.block(row, col, nbRows, Spline::NbCoeffs * rDof) *
        spline->rowParameters();
  }

  void linearize(const Splines_t& splines, const SplineOptimizationDatas_t& ss,
                 LinearConstraint& lc) {
    for (std::size_t i = 0; i < functions.size(); ++i)
      linearize(splines[splineIds[i]], ss[i], i, lc);
  }

  std::vector<typename CollisionFunction<SplinePtr_t>::Ptr_t> functions;
  std::vector<std::size_t> splineIds;
  std::vector<size_type> rows;
  std::vector<value_type> ratios;

  mutable Configuration_t q;
  mutable matrix_t J, Js;
  mutable typename Spline::BasisFunctionVector_t paramDerivativeCoeff;
};

// ----------- Resolution steps --------------------------------------- //

template <int _PB, int _SO>
typename SplineGradientBased<_PB, _SO>::Ptr_t
SplineGradientBased<_PB, _SO>::create(const ProblemConstPtr_t& problem) {
  SplineGradientBased* ptr = new SplineGradientBased(problem);
  Ptr_t shPtr(ptr);
  return shPtr;
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::addProblemConstraints(
    const PathVectorPtr_t& init, const Splines_t& splines, LinearConstraint& lc,
    SplineOptimizationDatas_t& ss) const {
  assert(init->numberPaths() == splines.size() && ss.size() == splines.size());
  for (std::size_t i = 0; i < splines.size(); ++i) {
    addProblemConstraintOnPath(init->pathAtRank(i), i, splines[i], lc, ss[i]);
  }
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::addProblemConstraintOnPath(
    const PathPtr_t& path, const size_type& idxSpline,
    const SplinePtr_t& spline, LinearConstraint& lc,
    SplineOptimizationData& sod) const {
  ConstraintSetPtr_t cs = path->constraints();
  if (cs) {
    ConfigProjectorPtr_t cp = cs->configProjector();
    if (cp) {
      const BySubstitution& hs = cp->solver();
      const ExplicitConstraintSet& es = hs.explicitConstraintSet();

      // Get the active parameter row selection.
      value_type guessThreshold =
          problem()
              ->getParameter("SplineGradientBased/guessThreshold")
              .floatValue();
      Eigen::RowBlockIndices select =
          computeActiveParameters(path, hs, guessThreshold);

      const size_type rDof = robot_->numberDof(),
                      col = idxSpline * Spline::NbCoeffs * rDof,
                      row = lc.J.rows(), nOutVar = select.nbIndices();

      sod.set = cs;
      sod.es.reset(new ExplicitConstraintSet(es));
      sod.activeParameters = RowBlockIndices(BlockIndex::difference(
          BlockIndex::segment_t(0, rDof), select.indices()));
      hppDout(info,
              "Path " << idxSpline << ": do not change this dof " << select);
      hppDout(info,
              "Path " << idxSpline << ": active dofs " << sod.activeParameters);

      // Add nOutVar constraint per coefficient.
      lc.addRows(Spline::NbCoeffs * nOutVar);
      matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
      for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
        lc.J.block(row + k * nOutVar, col + k * rDof, nOutVar, rDof) = I;
        lc.b.segment(row + k * nOutVar, nOutVar) =
            I * spline->parameters().row(k).transpose();
      }

      assert((lc.J.block(row, col, Spline::NbCoeffs * nOutVar,
                         rDof * Spline::NbCoeffs) *
              spline->rowParameters())
                 .isApprox(lc.b.segment(row, Spline::NbCoeffs * nOutVar)));
    }
  }
}

template <int _PB, int _SO>
Eigen::RowBlockIndices SplineGradientBased<_PB, _SO>::computeActiveParameters(
    const PathPtr_t& path, const BySubstitution& hs, const value_type& guessThr,
    const bool& useExplicitInput) const {
  const ExplicitConstraintSet& es = hs.explicitConstraintSet();

  BlockIndex::segments_t implicitBI, explicitBI;

  // Handle implicit part
  if (hs.reducedDimension() > 0) {
    implicitBI = hs.implicitDof();

    hppDout(info, "Solver " << hs << '\n'
                            << Eigen::RowBlockIndices(implicitBI));

    // in the case of PR2 passing a box from right to left hand,
    // the double grasp is a loop closure so the DoF of the base are
    // not active (one can see this in the Jacobian).
    // They should be left unconstrained.
    // TODO I do not see any good way of guessing this since it is
    // the DoF of the base are not active only on the submanifold
    // satisfying the constraint. It has to be dealt with in
    // hpp-manipulation.

    // If requested, check if the jacobian has columns of zeros.
    BlockIndex::segments_t passive;
    if (guessThr >= 0) {
      matrix_t J(hs.reducedDimension(), hs.freeVariables().nbIndices());
      hs.computeValue<true>(path->initial());
      hs.updateJacobian(path->initial());
      hs.getReducedJacobian(J);
      size_type j = 0, k = 0;
      for (size_type r = 0; r < J.cols(); ++r) {
        if (J.col(r).isZero(guessThr)) {
          size_type idof = es.notOutDers().indices()[j].first + k;
          passive.push_back(BlockIndex::segment_t(idof, 1));
          hppDout(info,
                  "Deactivated dof (thr=" << guessThr << ") " << idof
                                          << ". J = " << J.col(r).transpose());
        }
        k++;
        if (k >= hs.freeVariables().indices()[j].second) {
          j++;
          k = 0;
        }
      }
      BlockIndex::sort(passive);
      BlockIndex::shrink(passive);
      hppDout(info, "Deactivated dof (thr=" << guessThr << ") "
                                            << Eigen::ColBlockIndices(passive)
                                            << "J = " << J);
      implicitBI = BlockIndex::difference(implicitBI, passive);
    }
  } else if (useExplicitInput) {
    Eigen::ColBlockIndices esadp = es.activeDerivativeParameters();
    implicitBI = esadp.indices();
  }

  // Handle explicit part
  explicitBI = es.outDers().indices();

  // Add both
  implicitBI.insert(implicitBI.end(), explicitBI.begin(), explicitBI.end());
  Eigen::RowBlockIndices rbi(implicitBI);
  rbi.updateIndices<true, true, true>();
  return rbi;
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::addCollisionConstraint(
    const std::size_t idxSpline, const SplinePtr_t& spline,
    const SplinePtr_t& nextSpline, const SplineOptimizationData& sod,
    const PathValidationReportPtr_t& report, LinearConstraint& collision,
    CollisionFunctions& functions) const {
  hppDout(info, "Collision on spline "
                    << idxSpline << " at ratio (in [0,1]) = "
                    << report->parameter / nextSpline->length());
  typename CollisionFunction<SplinePtr_t>::Ptr_t cc(
      CollisionFunction<SplinePtr_t>::create(robot_, spline, nextSpline,
                                             report));

  collision.addRows(cc->outputSize());
  functions.addConstraint(cc, idxSpline, collision.J.rows() - 1,
                          report->parameter / nextSpline->length());

  functions.linearize(spline, sod, functions.functions.size() - 1, collision);
}

template <int _PB, int _SO>
bool SplineGradientBased<_PB, _SO>::findNewConstraint(
    LinearConstraint& constraint, LinearConstraint& collision,
    LinearConstraint& collisionReduced, CollisionFunctions& functions,
    const std::size_t iF, const SplinePtr_t& spline,
    const SplineOptimizationData& sod) const {
  HPP_SCOPE_TIMECOUNTER(SGB_findNewConstraint);
  bool solved = false;
  Configuration_t q(robot_->configSize());
  typename CollisionFunction<SplinePtr_t>::Ptr_t function(
      functions.functions[iF]);

  solved = constraint.reduceConstraint(collision, collisionReduced);

  size_type i = 5;
  while (not solved) {
    if (i == 0) {
      functions.removeLastConstraint(1, collision);
      hppDout(warning,
              "Could not find a suitable collision constraint. Removing it.");
      return false;
    }
    hppDout(info,
            "Looking for collision which does not make the constraint rank "
            "deficient.");
    // interpolate at alpha
    pinocchio::interpolate<pinocchio::RnxSOnLieGroupMap>(
        robot_, function->qFree_, function->qColl_, 0.5, q);
    hppDout(info, "New q: " << q.transpose());
    // update the constraint
    function->updateConstraint(q);
    functions.linearize(spline, sod, iF, collision);
    // check the rank
    solved = constraint.reduceConstraint(collision, collisionReduced, true);
    --i;
  }
  return true;
}

// ----------- Optimize ----------------------------------------------- //

template <int _PB, int _SO>
PathVectorPtr_t SplineGradientBased<_PB, _SO>::optimize(
    const PathVectorPtr_t& path) {
  this->monitorExecution();

  // Get some parameters
  value_type alphaInit =
      problem()->getParameter("SplineGradientBased/alphaInit").floatValue();
  bool alwaysStopAtFirst =
      problem()
          ->getParameter("SplineGradientBased/alwaysStopAtFirst")
          .boolValue();
  size_type costOrder =
      problem()->getParameter("SplineGradientBased/costOrder").intValue();
  bool usePathLengthAsWeights =
      problem()
          ->getParameter("SplineGradientBased/usePathLengthAsWeights")
          .boolValue();
  bool reorderIntervals =
      problem()
          ->getParameter("SplineGradientBased/reorderIntervals")
          .boolValue();
  bool linearizeAtEachStep =
      problem()
          ->getParameter("SplineGradientBased/linearizeAtEachStep")
          .boolValue();
  bool checkJointBound =
      problem()
          ->getParameter("SplineGradientBased/checkJointBound")
          .boolValue();
  bool returnOptimum =
      problem()->getParameter("SplineGradientBased/returnOptimum").boolValue();
  value_type costThreshold =
      problem()->getParameter("SplineGradientBased/costThreshold").floatValue();
  bool useProxqp =
    problem()->getParameter("SplineGradientBased/useProxqp").boolValue();
  value_type eps_abs(problem()->getParameter("SplineGradientBased/QPAccuracy").
                     floatValue());
  if (path->length() == 0) return path;
  PathVectorPtr_t input = Base::cleanInput(path);

  const size_type rDof = robot_->numberDof();

  // 1
  // Replace each path of the vector by a spline with 0 derivatives at
  // start and end.
  Splines_t splines;
  this->appendEquivalentSpline(input, splines);
  const size_type nParameters = splines.size() * Spline::NbCoeffs;

  // Initialize one path validation method for each spline.
  // Path validation methods are retrieve in the transition of the
  // constraint graph that produced the initial part of the path.
  std::vector<std::size_t> collisionReordering(splines.size());
  for (std::size_t i = 0; i < splines.size(); ++i) collisionReordering[i] = i;
  this->initializePathValidation(splines);

  // 2
  enum { MaxContinuityOrder = int((SplineOrder - 1) / 2) };
  const size_type orderContinuity = MaxContinuityOrder;

  LinearConstraint constraint(nParameters * rDof, 0);
  SplineOptimizationDatas_t solvers(splines.size(),
                                    SplineOptimizationData(rDof));
  addProblemConstraints(input, splines, constraint, solvers);

  this->addContinuityConstraints(splines, orderContinuity, solvers, constraint);

  // 3
  LinearConstraint collision(nParameters * rDof, 0);
  CollisionFunctions collisionFunctions;

  // 4
  L2NormSquaredOfDerivative<Spline> cost(splines, rDof, rDof, costOrder);
  if (usePathLengthAsWeights) {
    cost.computeLambdasFromSplineLength(splines);
  }

  // 5
  //
  // true = check that the constraint is feasible.
  // true = throws if the constraint is infeasible.
  constraint.decompose(true, true);

  LinearConstraint collisionReduced(constraint.PK.rows(), 0);
  constraint.reduceConstraint(collision, collisionReduced);

  LinearConstraint boundConstraint(nParameters * rDof, 0);
  if (checkJointBound) {
    this->jointBoundConstraint(splines, boundConstraint);
    if (!this->validateBounds(splines, boundConstraint).empty())
      throw std::invalid_argument("Input path does not satisfy joint bounds");
  }
  LinearConstraint boundConstraintReduced(constraint.PK.rows(), 0);
  constraint.reduceConstraint(boundConstraint, boundConstraintReduced, false);

  // 6
  bool noCollision = true, stopAtFirst = alwaysStopAtFirst;
  bool minimumReached = false;

  bool computeOptimum = true,
       computeInterpolatedSpline = !(checkOptimum_ || returnOptimum);

  value_type alpha = (checkOptimum_ || returnOptimum ? 1 : alphaInit);

  Splines_t alphaSplines, collSplines;
  Splines_t* currentSplines(0x0);
  Base::copy(splines, alphaSplines);
  Base::copy(splines, collSplines);
  Reports_t reports;

  QuadraticProgram QP(cost.inputDerivativeSize_, useProxqp);
  QP.accuracy(eps_abs);
  value_type optimalCost, costLowerBound = 0;
  cost.value(optimalCost, splines);
  hppDout(info, "Initial cost is " << optimalCost);
  cost.hessian(QP.H, splines);
#ifndef NDEBUG
  checkHessian(cost, QP.H, splines);
#endif  // NDEBUG

  QuadraticProgram QPc(QP, constraint, useProxqp);
  QPc.accuracy(eps_abs);

  if (QPc.H.rows() == 0)
    // There are no variables left for optimization.
    return this->buildPathVector(splines);
  QPc.computeLLT();
  QPc.solve(collisionReduced, boundConstraintReduced);

  while (!(noCollision && minimumReached) && !this->shouldStop()) {
    // 6.1
    if (computeOptimum) {
      // 6.2
      constraint.computeSolution(QPc.xStar);
      Base::updateSplines(collSplines, constraint.xSol);
      cost.value(costLowerBound, collSplines);
      hppDout(info, "Cost interval: [" << costLowerBound << ", " << optimalCost
                                       << "]");
      currentSplines = &collSplines;
      minimumReached = true;
      computeOptimum = false;
    }
    if (computeInterpolatedSpline) {
      Base::interpolate(splines, collSplines, alpha, alphaSplines);
      currentSplines = &alphaSplines;
      minimumReached = false;
      computeInterpolatedSpline = false;
    }

    // 6.3.2 Check for collision
    if (!returnOptimum) {
      reports = this->validatePath(*currentSplines, collisionReordering,
                                   stopAtFirst, reorderIntervals);
      noCollision = reports.empty();
    } else {
      minimumReached = true;
      noCollision = true;
    }
    if (noCollision) {
      cost.value(optimalCost, *currentSplines);
      hppDout(info, "Cost interval: [" << costLowerBound << ", " << optimalCost
                                       << "]");
      // Update the spline
      for (std::size_t i = 0; i < splines.size(); ++i)
        splines[i]->rowParameters((*currentSplines)[i]->rowParameters());
      if (linearizeAtEachStep) {
        collisionFunctions.linearize(splines, solvers, collision);
        constraint.reduceConstraint(collision, collisionReduced);
        QPc.solve(collisionReduced, boundConstraintReduced);
        hppDout(info, "linearized");
        computeOptimum = true;
      }
      hppDout(info, "Improved path with alpha = " << alpha);

      computeInterpolatedSpline = true;
      if (!minimumReached && std::abs(optimalCost - costLowerBound) <
                                 costThreshold * costLowerBound) {
        hppDout(info, "Stopping because cost interval is small.");
        minimumReached = true;
      }
    } else {
      if (alpha != 1.) {
        if (QPc.H.rows() <= collisionReduced.rank) {
          hppDout(info, "No more constraints can be added."
                            << QP.H.rows() << " variables for "
                            << collisionReduced.rank
                            << " independant constraints.");
          break;
        }

        bool ok = false;
        for (std::size_t i = 0; i < reports.size(); ++i) {
          addCollisionConstraint(reports[i].second, splines[reports[i].second],
                                 (*currentSplines)[reports[i].second],
                                 solvers[reports[i].second], reports[i].first,
                                 collision, collisionFunctions);

          ok |= findNewConstraint(
              constraint, collision, collisionReduced, collisionFunctions,
              collisionFunctions.functions.size() - 1,
              splines[reports[i].second], solvers[reports[i].second]);
          if (!ok) break;
          if (QPc.H.rows() <= collisionReduced.rank) break;
        }

        if (!ok) {
          hppDout(info,
                  "The collision constraint would be rank deficient. Removing "
                  "added constraint.");
          if (alpha < alphaInit / (1 << 2)) {
            hppDout(info, "Interruption because alpha became too small.");
            break;
          }
          alpha *= 0.5;
          stopAtFirst = alwaysStopAtFirst;

          computeInterpolatedSpline = true;
        } else {
          QPc.solve(collisionReduced, boundConstraintReduced);
          hppDout(info, "Added " << reports.size()
                                 << " constraints. "
                                    "Constraints size "
                                 << collision.J.rows()
                                 << "(rank=" << collisionReduced.rank
                                 << ", ass=" << QPc.activeSetSize << ") / "
                                 << QPc.H.cols());

          // When adding a new constraint, try first minimum under this
          // constraint. If this latter minimum is in collision,
          // re-initialize alpha to alphaInit.
          alpha = 1.;
          stopAtFirst = true;
          computeOptimum = true;
        }
      } else {
        alpha = alphaInit;
        stopAtFirst = alwaysStopAtFirst;
        computeInterpolatedSpline = true;
      }
    }
    this->endIteration();
  }

  // 7
  HPP_DISPLAY_TIMECOUNTER(SGB_findNewConstraint);
  return this->buildPathVector(splines);
}

// ----------- Convenience functions ---------------------------------- //

template <int _PB, int _SO>
template <typename Cost_t>
bool SplineGradientBased<_PB, _SO>::checkHessian(
    const Cost_t& cost, const matrix_t& H, const Splines_t& splines) const {
  value_type expected;
  cost.value(expected, splines);

  vector_t P(H.rows());

  const size_type size = robot_->numberDof() * Spline::NbCoeffs;
  for (std::size_t i = 0; i < splines.size(); ++i)
    P.segment(i * size, size) = splines[i]->rowParameters();
  value_type result = 0.5 * P.transpose() * H * P;

  bool ret = std::fabs(expected - result) <
             Eigen::NumTraits<value_type>::dummy_precision();
  if (!ret) {
    hppDout(error, "Hessian of the cost is not correct: " << expected << " - "
                                                          << result << " = "
                                                          << expected - result);
  }
  return ret;
}

// ----------- Instanciate -------------------------------------------- //

// template class SplineGradientBased<path::CanonicalPolynomeBasis, 1>; //
// equivalent to StraightPath template class
// SplineGradientBased<path::CanonicalPolynomeBasis, 2>; template class
// SplineGradientBased<path::CanonicalPolynomeBasis, 3>;
template class SplineGradientBased<path::BernsteinBasis,
                                   1>;  // equivalent to StraightPath
// template class SplineGradientBased<path::BernsteinBasis, 2>;
template class SplineGradientBased<path::BernsteinBasis, 3>;
template class SplineGradientBased<path::BernsteinBasis, 5>;
template class SplineGradientBased<path::BernsteinBasis, 7>;

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(SplineGradientBased)
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "SplineGradientBased/alphaInit",
                         "In [0,1]. The initial value used when interpolating "
                         "between non-colliding current solution and"
                         " the optimal colliding trajector.",
                         Parameter(0.2)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/alwaysStopAtFirst",
    "If true, consider only one (not all) collision constraint at each "
    "iteration.",
    Parameter(true)));
Problem::declareParameter(ParameterDescription(
    Parameter::INT, "SplineGradientBased/costOrder",
    "The order of the derivative used for the optimized cost function. This is "
    "most likely 1, 2 or 3",
    Parameter((size_type)1)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/usePathLengthAsWeights",
    "If true, the initial path length are used to weight the splines.",
    Parameter(false)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/reorderIntervals",
    "If true, interval in collision are checked first at next iteration.",
    Parameter(false)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/linearizeAtEachStep",
    "If true, collision constraint will be re-linearized at each iteration.",
    Parameter(false)));
Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/checkJointBound",
    "If true, joint bounds are enforced.", Parameter(true)));
Problem::declareParameter(
    ParameterDescription(Parameter::BOOL, "SplineGradientBased/returnOptimum",
                         "(for debugging purpose) If true, returns the optimum "
                         "regardless of collision.",
                         Parameter(false)));
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "SplineGradientBased/costThreshold",
                         "Stop optimizing if the cost improves less than this "
                         "threshold between two iterations.",
                         Parameter(0.01)));
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "SplineGradientBased/guessThreshold",
                         "Threshold used to check whether the Jacobian "
                         "contains rows of zeros, in which case the "
                         "corresponding DoF is considered passive.",
                         Parameter(-1.)));
Problem::declareParameter(
    ParameterDescription(Parameter::BOOL, "SplineGradientBased/useProxqp",
        "Use proxqp QP solver instead of eiquadprog_2011. Temporary parameter "
        "that will be removed soon.", Parameter(true)));
Problem::declareParameter(
    ParameterDescription(Parameter::FLOAT, "SplineGradientBased/QPAccuracy",
        "Accuracy of QP solver (only used by proxqp.", Parameter(1e-4)));
HPP_END_PARAMETER_DECLARATION(SplineGradientBased)
}  // namespace pathOptimization
}  // namespace core
}  // namespace hpp
