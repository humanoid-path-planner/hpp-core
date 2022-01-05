// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//          Olivier Roussel (olivier.roussel@laas.fr)
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

#include "toppra.hh"

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/plugin.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/time-parameterization/piecewise-polynomial.hh>

#include <toppra/toppra.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/solver/glpk-wrapper.hpp>
#include <toppra/solver/qpOASES-wrapper.hpp>
#include <toppra/solver/seidel.hpp>

#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/joint_torque/pinocchio.hpp>

namespace hpp {
namespace core {

class PathWrapper : public toppra::GeometricPath
{
public:
  PathWrapper(PathPtr_t path)
      : toppra::GeometricPath((int)path->outputSize(), (int)path->outputDerivativeSize()), path_(path)
  {
  }

  toppra::Vector eval_single(toppra::value_type time, int order) const
  {
    bool success;
    toppra::Vector res;
    if (order == 0)
    {
      res = path_->eval(time, success);
      assert(success);
    }
    else
    {
      res.resize(dof());
      path_->derivative(res, time, order);
    }
    return res;
  }

  toppra::Bound pathInterval() const
  {
    const interval_t &tr = path_->timeRange();
    return (toppra::Bound() << tr.first, tr.second).finished();
  }

private:
  PathPtr_t path_;
};

namespace timeParameterization
{
class Extract : public TimeParameterization
{
  public:
  Extract(TimeParameterizationPtr_t inner, value_type dt, value_type ds)
    : inner_(inner), dt_(dt), ds_ (ds)
  {}

  value_type value (const value_type& t) const
  {
    return inner_->value(t+dt_) + ds_;
  }

  value_type derivative (const value_type& t, const size_type& order) const
  {
    return inner_->derivative(t+dt_, order);
  }

  value_type derivativeBound (const value_type& low, const value_type& up) const
  {
    return derivativeBound (low+dt_, up+dt_);
  }

  TimeParameterizationPtr_t copy () const
  {
    return TimeParameterizationPtr_t(new Extract(inner_, dt_, ds_));
  }

  private:
  TimeParameterizationPtr_t inner_;
  value_type dt_, ds_;
};
}

#define PARAM_HEAD "PathOptimization/TOPPRA/"

namespace pathOptimization
{
PathVectorPtr_t TOPPRA::optimize(const PathVectorPtr_t &path)
{
  const size_type solver = problem()->getParameter
    (PARAM_HEAD "solver").intValue();
  const value_type effortScale = problem()->getParameter
    (PARAM_HEAD "effortScale").floatValue();
  const value_type velScale = problem()->getParameter
    (PARAM_HEAD "velocityScale").floatValue();

  using pinocchio::Model;
  const Model &model = problem()->robot()->model();

  using namespace toppra::constraint;

  // Create the TOPPRA constraints
  auto torqueConstraint = std::make_shared<jointTorque::Pinocchio<Model> > (model); // No friction
  torqueConstraint->lowerBounds(effortScale * torqueConstraint->lowerBounds());
  torqueConstraint->upperBounds(effortScale * torqueConstraint->upperBounds());

  toppra::LinearConstraintPtrs v{
    std::make_shared<LinearJointVelocity>(
        -velScale * model.velocityLimit, velScale * model.velocityLimit)
      //, std::make_shared<LinearJointAcceleration>(
      //- 10 * model.velocityLimit, 10 * model.velocityLimit)
      , torqueConstraint
  };

  PathVectorPtr_t flatten_path = PathVector::create(path->outputSize(), path->outputDerivativeSize());
  path->flatten(flatten_path);

  const value_type min_path_length = 1e-6;
  if (path->length() < min_path_length)
  {
    return flatten_path;
  }

  size_type N = problem()->getParameter(PARAM_HEAD "N").intValue();

  // 1. Compute TOPPRA grid points (in the parameter space).
  std::vector<value_type> params;
  params.reserve(N + flatten_path->numberPaths() - 1);
  value_type dparams = flatten_path->length() / (value_type)N;
  std::vector<size_type> id_subpaths;
  id_subpaths.reserve(flatten_path->numberPaths()+1);

  std::vector<PathPtr_t> paths (flatten_path->numberPaths());
  params.push_back(0.);
  id_subpaths.push_back(0);
  for(auto i = 0ul; i < flatten_path->numberPaths(); ++i)
  {
    const auto subpath = (paths[i] = flatten_path->pathAtRank(i));
    auto I = subpath->paramRange();
    size_type n = size_type(std::ceil((I.second-I.first) / dparams));
    value_type p0 = params.back();
    for (size_type k = 1; k <= n; ++k)
      params.push_back(p0 + ((I.second-I.first) * (value_type)k ) / (value_type)n);
    id_subpaths.push_back(params.size() - 1);
  }
  toppra::Vector gridpoints (Eigen::Map<toppra::Vector>(params.data(), params.size()));
  N = gridpoints.size() - 1;

  // 2. Apply TOPPRA on the full path
  std::shared_ptr<PathWrapper> pathWrapper (std::make_shared<PathWrapper>(flatten_path));
  toppra::algorithm::TOPPRA algo(v, pathWrapper);
  algo.setN((int)N);
  switch(solver) {
    default:
      hppDout (error, "Solver " << solver << " does not exists. Using Seidel");
    case 0:
      algo.solver(std::make_shared<toppra::solver::Seidel>());
      break;
    case 1:
      algo.solver(std::make_shared<toppra::solver::GLPKWrapper>());
      break;
    case 2:
      algo.solver(std::make_shared<toppra::solver::qpOASESWrapper>());
      break;
  }
  algo.setGridpoints(gridpoints);
  auto ret_code = algo.computePathParametrization();
  if (ret_code != toppra::ReturnCode::OK)
  {
    std::stringstream ss;
    ss << "TOPPRA failed, returned code: " << static_cast<int>(ret_code) << std::endl;
    throw std::runtime_error(ss.str());
  }
  const auto out_data = algo.getParameterizationData();
  assert(out_data.gridpoints.size() == out_data.parametrization.size());
  toppra::Vector sd (out_data.parametrization.cwiseMax(0.).cwiseSqrt());

  // 3. Build the parameterization function and apply it on each subpaths.
  PathVectorPtr_t res = PathVector::create(path->outputSize(), path->outputDerivativeSize());

  // 3.1 forward integration of time parameterization (trapezoidal integration)
  vector_t t = vector_t(N+1);
  t[0] = 0.; // start time is 0
  for (size_type i = 1; i <= N; ++i)
  {
    const auto sd_avg = (sd[i - 1] + sd[i]) * 0.5;
    const auto ds = out_data.gridpoints[i] - out_data.gridpoints[i - 1];
    assert(sd_avg > 0.);
    const auto dt = ds / sd_avg;    
    t[i] = t[i - 1] + dt;
  }

  // 3.2 time parameterization based on hermite cubic spline interpolation
  constexpr int order = 3;
  typedef timeParameterization::PiecewisePolynomial<order> timeparam;
  auto spline_coeffs = timeparam::ParameterMatrix_t(order + 1, N);
  const auto& s = out_data.gridpoints;
  for (size_type i = 1; i <= N; ++i)
  {
    const auto inv_dt = 1./(t[i] - t[i-1]);
    const auto inv_dt2 = inv_dt*inv_dt;
    const auto inv_dt3 = inv_dt2*inv_dt;
    const auto t_p = t[i-1];
    const auto t_p2 = t_p*t_p;
    const auto t_p3 = t_p2*t_p;
    const auto ds = (s[i] - s[i-1]);
    const auto b = (2*sd[i-1] + sd[i])*inv_dt;
    const auto c = (sd[i-1] + sd[i])*inv_dt2;
    spline_coeffs(0, i-1) = 2*t_p3*ds*inv_dt3 + 3*t_p2*ds*inv_dt2 - t_p3*c -t_p2*b - t_p*sd[i-1] + s[i-1];
    spline_coeffs(1, i-1) = -6*t_p2*ds*inv_dt3 - 6*t_p*ds*inv_dt2 + 3*t_p2*c + 2*t_p*b + sd[i-1];
    spline_coeffs(2, i-1) = 6*t_p*ds*inv_dt3 + 3*ds*inv_dt2 - 3*t_p*c - b;
    spline_coeffs(3, i-1) = -2*ds*inv_dt3 + c;
  }

  TimeParameterizationPtr_t global (new timeparam(spline_coeffs, t));
  for(auto i = 0ul; i < paths.size(); ++i)
  {
    value_type t0 = t[id_subpaths[i]];
    value_type p0 = -gridpoints[id_subpaths[i]];
    paths[i]->timeParameterization(
        TimeParameterizationPtr_t(new timeParameterization::Extract (global, t0, p0)),
        interval_t(0, t[id_subpaths[i+1]]-t0));
    res->appendPath(paths[i]);

  }

  return res;
}

HPP_START_PARAMETER_DECLARATION(TOPPRA)
Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
      PARAM_HEAD "effortScale",
      "Effort rescaling value.",
      Parameter((value_type)1)));
Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
      PARAM_HEAD "velocityScale",
      "Velocity rescaling value.",
      Parameter((value_type)1)));
Problem::declareParameter(ParameterDescription (Parameter::INT,
      PARAM_HEAD "solver",
      "0: GLPK\n"
      "1: qpOASES",
      Parameter((size_type)1)));
Problem::declareParameter(ParameterDescription (Parameter::INT,
      PARAM_HEAD "N",
      "Number of sampling point.",
      Parameter((size_type)50)));
HPP_END_PARAMETER_DECLARATION(TOPPRA)
} // namespace pathOptimization

class TOPPRAPlugin : public ProblemSolverPlugin
{
public:
  TOPPRAPlugin()
      : ProblemSolverPlugin("TOPPRAPlugin", "0.0")
  {
  }

protected:
  virtual bool impl_initialize(ProblemSolverPtr_t ps)
  {
    ps->pathOptimizers.add("TOPPRA", pathOptimization::TOPPRA::create);
    return true;
  }
};
} // namespace core
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::core::TOPPRAPlugin)
