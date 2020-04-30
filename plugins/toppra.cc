// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <toppra/toppra.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/algorithm/toppra.hpp>

#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/joint_torque/pinocchio.hpp>

namespace hpp {
namespace core {

class PathWrapper : public toppra::GeometricPath
{
  public:
    PathWrapper(PathPtr_t path)
      : toppra::GeometricPath ((int)path->outputSize(), (int)path->outputDerivativeSize())
      , path_ (path)
    {}

    toppra::Vector eval_single(toppra::value_type time, int order) const
    {
      bool success;
      toppra::Vector res;
      if (order == 0) {
        res = path_->eval(time, success);
        assert(success);
      } else {
        res.resize(dof());
        path_->derivative(res, time, order);
      }
      return res;
    }

    toppra::Bound pathInterval() const
    {
      const interval_t& tr = path_->timeRange();
      return (toppra::Bound() << tr.first, tr.second).finished();
    }

  private:
    PathPtr_t path_;
};

namespace pathOptimization {
class TOPPRA;
typedef boost::shared_ptr<TOPPRA> TOPPRAPtr_t;

class TOPPRA : public PathOptimizer
{
  public:
    static TOPPRAPtr_t create (const Problem& p)
    {
      return TOPPRAPtr_t(new TOPPRA(p));
    }

    PathVectorPtr_t optimize (const PathVectorPtr_t& path)
    {
      using pinocchio::Model;
      const Model& model = problem().robot()->model();

      using namespace toppra::constraint;

      // Create the TOPPRA constraints
      toppra::LinearConstraintPtrs v{
        std::make_shared<LinearJointVelocity>(
            - model.velocityLimit, model.velocityLimit),
          std::make_shared<LinearJointAcceleration>(
              - model.effortLimit, model.effortLimit),
          std::make_shared<jointTorque::Pinocchio<Model> >(
              model, toppra::Vector::Constant(model.nv, 0.05))
      };
      PathWrapper pathWrapper (path);

      toppra::algorithm::TOPPRA algo (v, pathWrapper);
      algo.setN(50);
      auto ret_code = algo.computePathParametrization();
      if (ret_code != toppra::ReturnCode::OK)
        throw std::runtime_error("TOPPRA failed.");
      // TODO convert toppra result into TimeParameterization object.
      return path;
    }

  protected:
    TOPPRA (const Problem& problem) : PathOptimizer (problem)
    {}
};
}

class TOPPRAPlugin : public ProblemSolverPlugin
{
  public:
    TOPPRAPlugin ()
      : ProblemSolverPlugin ("TOPPRAPlugin", "0.0")
    {}

  protected:
    virtual bool impl_initialize (ProblemSolverPtr_t ps)
    {
      ps->pathOptimizers.add("TOPPRA", pathOptimization::TOPPRA::create);
      return true;
    }
};
} // namespace core
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::core::TOPPRAPlugin)
