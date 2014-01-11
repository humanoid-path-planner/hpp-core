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

#ifndef HPP_CORE_FWD_HH
# define HPP_CORE_FWD_HH


# include <vector>
# include <deque>
# include <list>
# include <hpp/util/pointer.hh>
# include <roboptim/core/function.hh>
# include <hpp/model/fwd.hh>

namespace roboptim {
  class StableTimePoint;
} // namespace roboptim

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (BasicConfigurationShooter);
    HPP_PREDEF_CLASS (ConfigurationShooter);
    HPP_PREDEF_CLASS (ConfigProjector);
    HPP_PREDEF_CLASS (ConnectedComponent);
    HPP_PREDEF_CLASS (Constraint);
    HPP_PREDEF_CLASS (ConstraintSet);
    HPP_PREDEF_CLASS (DifferentiableFunction);
    HPP_PREDEF_CLASS (DiffusingPlanner);
    HPP_PREDEF_CLASS (Distance);
    HPP_PREDEF_CLASS (DiscretizedCollisionChecking);
    class Edge;
    HPP_PREDEF_CLASS (ExtractedPath);
    HPP_PREDEF_CLASS (LockedDof);
    class Node;
    HPP_PREDEF_CLASS (Path);
    HPP_PREDEF_CLASS (PathOptimizer);
    HPP_PREDEF_CLASS (PathPlanner);
    HPP_PREDEF_CLASS (PathVector);
    HPP_PREDEF_CLASS (PathValidation);
    HPP_PREDEF_CLASS (PlanAndOptimize);
    HPP_PREDEF_CLASS (Problem);
    class ProblemSolver;
    HPP_PREDEF_CLASS (RandomShortcut);
    HPP_PREDEF_CLASS (Roadmap);
    HPP_PREDEF_CLASS (SteeringMethod);
    HPP_PREDEF_CLASS (SteeringMethodStraight);
    HPP_PREDEF_CLASS (StraightPath);
    HPP_PREDEF_CLASS (WeighedDistance);

    // roboptim
    typedef roboptim::GenericFunctionTraits
    <roboptim::EigenMatrixDense>::value_type value_type;
    typedef BasicConfigurationShooterShPtr BasicConfigurationShooterPtr_t;
    typedef model::CollisionObjectShPtr CollisionObjectShPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef boost::shared_ptr<model::Configuration_t> ConfigurationPtr_t;
    typedef std::vector <ConfigurationPtr_t> Configurations_t;
    typedef Configurations_t::iterator ConfigIterator_t;
    typedef Configurations_t::const_iterator ConfigConstIterator_t;
    typedef ConfigurationShooterShPtr ConfigurationShooterPtr_t;
    typedef ConfigProjectorShPtr ConfigProjectorPtr_t;
    typedef ConnectedComponentShPtr ConnectedComponentPtr_t;
    typedef std::list < ConnectedComponentPtr_t > ConnectedComponents_t;
    typedef ConstraintShPtr ConstraintPtr_t;
    typedef ConstraintSetShPtr ConstraintSetPtr_t;
    typedef model::Device Device_t;
    typedef model::DeviceShPtr DevicePtr_t;
    typedef model::DeviceWkPtr DeviceWkPtr_t;
    typedef std::deque < DevicePtr_t > Devices_t;
    typedef DifferentiableFunctionShPtr DifferentiableFunctionPtr_t;
    typedef DiffusingPlannerShPtr DiffusingPlannerPtr_t;
    typedef DiscretizedCollisionCheckingShPtr DiscretizedCollisionCheckingPtr_t;
    typedef DistanceShPtr DistancePtr_t;
    typedef Edge* EdgePtr_t;
    typedef std::list < Edge* > Edges_t;
    typedef ExtractedPathShPtr ExtractedPathPtr_t;
    typedef model::JointJacobian_t JointJacobian_t;
    typedef model::HalfJointJacobian_t HalfJointJacobian_t;
    typedef model::JointVector_t JointVector_t;
    typedef LockedDofShPtr LockedDofPtr_t;
    typedef model::matrix_t matrix_t;
    typedef std::list < Node* > Nodes_t;
    typedef std::list < Node* > Nodes_t;
    typedef Node* NodePtr_t;
    typedef model::ObjectVector_t ObjectVector_t;
    typedef PathShPtr PathPtr_t;
    typedef PathOptimizerShPtr PathOptimizerPtr_t;
    typedef PathPlannerShPtr PathPlannerPtr_t;
    typedef PathValidationShPtr PathValidationPtr_t;
    typedef PathVectorShPtr PathVectorPtr_t;
    typedef PlanAndOptimizeShPtr PlanAndOptimizePtr_t;
    typedef Problem* ProblemPtr_t;
    typedef ProblemSolver* ProblemSolverPtr_t;
    typedef RandomShortcutShPtr RandomShortcutPtr_t;
    typedef RoadmapShPtr RoadmapPtr_t;
    typedef roboptim::Function::size_type size_type;
    typedef StraightPathShPtr StraightPathPtr_t;
    typedef SteeringMethodShPtr SteeringMethodPtr_t;
    typedef SteeringMethodStraightShPtr SteeringMethodStraightPtr_t;
    typedef roboptim::StableTimePoint StableTimePoint_t;
    typedef std::vector <PathPtr_t> Paths_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef model::vector_t vector_t;
    typedef WeighedDistanceShPtr WeighedDistancePtr_t;
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_FWD_HH
