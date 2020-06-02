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

#define PARAM_HEAD "PathOptimization/TOPPRA/"

namespace pathOptimization
{
class TOPPRA;
typedef boost::shared_ptr<TOPPRA> TOPPRAPtr_t;

class TOPPRA : public PathOptimizer
{
  public:
    static TOPPRAPtr_t create(const Problem &p)
    {
      return TOPPRAPtr_t(new TOPPRA(p));
    }

  PathVectorPtr_t optimize(const PathVectorPtr_t &path)
  {
    const size_type solver = problem().getParameter(PARAM_HEAD "solver").intValue();
    const value_type effortScale = problem().getParameter(PARAM_HEAD "effortScale").floatValue();
    const value_type velScale = problem().getParameter(PARAM_HEAD "velocityScale").floatValue();

    using pinocchio::Model;
    const Model &model = problem().robot()->model();

    using namespace toppra::constraint;

    // Create the TOPPRA constraints
    auto torqueConstraint = std::make_shared<jointTorque::Pinocchio<Model> >
      (model, toppra::Vector::Constant(model.nv, 0.)); // No friction
    torqueConstraint->lowerBounds(effortScale * torqueConstraint->lowerBounds());
    torqueConstraint->upperBounds(effortScale * torqueConstraint->upperBounds());

    toppra::LinearConstraintPtrs v{
        std::make_shared<LinearJointVelocity>(
            -velScale * model.velocityLimit, velScale * model.velocityLimit)
        //, std::make_shared<LinearJointAcceleration>(
            //- 10 * model.velocityLimit, 10 * model.velocityLimit)
        , torqueConstraint
            };
    std::shared_ptr<PathWrapper> pathWrapper (std::make_shared<PathWrapper>(path));

    toppra::algorithm::TOPPRA algo(v, pathWrapper);
    algo.setN((int)problem().getParameter(PARAM_HEAD "N").intValue());
    switch(solver) {
      default:
        hppDout (error, "Solver " << solver << " does not exists. Using GLPK");
      case 0:
        algo.solver(std::make_shared<toppra::solver::GLPKWrapper>());
        break;
      case 1:
        algo.solver(std::make_shared<toppra::solver::qpOASESWrapper>());
        break;
    }
    auto ret_code = algo.computePathParametrization();
    if (ret_code != toppra::ReturnCode::OK)
    {
      std::stringstream ss;
      ss << "TOPPRA failed, returned code: " << static_cast<int>(ret_code) << std::endl;
      throw std::runtime_error(ss.str());
    }
    const auto out_data = algo.getParameterizationData();

    // forward integration of time parameterization (trapezoidal integration)
    assert(out_data.gridpoints.size() == out_data.parametrization.size());
    const size_t num_pts = out_data.gridpoints.size();
    auto sd = toppra::Vector(num_pts);
    for (auto i = 0ul; i < num_pts; ++i)
    {
      sd[i] = std::sqrt(std::max(out_data.parametrization[i], 0.));
    }

    auto t = toppra::Vector(num_pts);
    t[0] = 0.; // start time is 0
    for (auto i = 1ul; i < num_pts; ++i)
    {
      const auto sd_avg = (sd[i - 1] + sd[i]) * 0.5;
      const auto ds = out_data.gridpoints[i] - out_data.gridpoints[i - 1];
      assert(sd_avg > 0.);
      const auto dt = ds / sd_avg;    
      t[i] = t[i - 1] + dt;
    }

    // time parameterization based on hermite cubic spline interpolation
    constexpr int order = 3;
    typedef timeParameterization::PiecewisePolynomial<order> timeparm;
    auto params = timeparm::ParameterMatrix_t(order + 1, num_pts - 1);
    const auto& s = out_data.gridpoints;
    for (auto i = 1ul; i < num_pts; ++i)
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
      params(0, i-1) = 2*t_p3*ds*inv_dt3 + 3*t_p2*ds*inv_dt2 - t_p3*c -t_p2*b - t_p*sd[i-1] + s[i-1];
      params(1, i-1) = -6*t_p2*ds*inv_dt3 - 6*t_p*ds*inv_dt2 + 3*t_p2*c + 2*t_p*b + sd[i-1];
      params(2, i-1) = 6*t_p*ds*inv_dt3 + 3*ds*inv_dt2 - 3*t_p*c - b;
      params(3, i-1) = -2*ds*inv_dt3 + c;
    }


    PathVectorPtr_t res = PathVector::createCopy(path);
    res->timeParameterization(TimeParameterizationPtr_t(new timeparm(params, t)),
                              interval_t(t[0], t[num_pts - 1]));

    return res;
  }

protected:
  TOPPRA(const Problem &problem) : PathOptimizer(problem)
  {
  }
};

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
