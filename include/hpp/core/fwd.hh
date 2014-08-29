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
# include <set>
# include <hpp/util/pointer.hh>
# include <roboptim/core/function.hh>
# include <hpp/model/fwd.hh>

namespace roboptim {
  class StableTimePoint;
} // namespace roboptim

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (BasicConfigurationShooter);
    HPP_PREDEF_CLASS (CollisionValidation);
    HPP_PREDEF_CLASS (ConfigurationShooter);
    HPP_PREDEF_CLASS (ConfigProjector);
    HPP_PREDEF_CLASS (ConfigValidation);
    HPP_PREDEF_CLASS (ConfigValidations);
    HPP_PREDEF_CLASS (ConnectedComponent);
    HPP_PREDEF_CLASS (Constraint);
    HPP_PREDEF_CLASS (ConstraintSet);
    HPP_PREDEF_CLASS (DifferentiableFunction);
    HPP_PREDEF_CLASS (DiffusingPlanner);
    HPP_PREDEF_CLASS (Distance);
    HPP_PREDEF_CLASS (DiscretizedCollisionChecking);
    class Edge;
    HPP_PREDEF_CLASS (ExtractedPath);
    HPP_PREDEF_CLASS (JointBoundValidation);
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
    class KDTree;

    // roboptim
    typedef roboptim::GenericFunctionTraits
    <roboptim::EigenMatrixDense>::value_type value_type;
    typedef roboptim::Function::interval_t interval_t;
    typedef roboptim::Function::size_type size_type;

    typedef boost::shared_ptr < BasicConfigurationShooter >
    BasicConfigurationShooterPtr_t;
    typedef boost::shared_ptr <CollisionValidation> CollisionValidationPtr_t;
    typedef model::CollisionObjectPtr_t CollisionObjectPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef boost::shared_ptr<model::Configuration_t> ConfigurationPtr_t;
    typedef std::vector <ConfigurationPtr_t> Configurations_t;
    typedef Configurations_t::iterator ConfigIterator_t;
    typedef Configurations_t::const_iterator ConfigConstIterator_t;
    typedef boost::shared_ptr <ConfigurationShooter> ConfigurationShooterPtr_t;
    typedef boost::shared_ptr <ConfigProjector> ConfigProjectorPtr_t;
    typedef boost::shared_ptr <ConfigValidation> ConfigValidationPtr_t;
    typedef boost::shared_ptr <ConfigValidations> ConfigValidationsPtr_t;
    typedef boost::shared_ptr <ConnectedComponent> ConnectedComponentPtr_t;
    typedef std::set <ConnectedComponentPtr_t> ConnectedComponents_t;
    typedef boost::shared_ptr <Constraint> ConstraintPtr_t;
    typedef boost::shared_ptr <ConstraintSet> ConstraintSetPtr_t;
    typedef model::Device Device_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::DeviceWkPtr_t DeviceWkPtr_t;
    typedef std::deque <DevicePtr_t> Devices_t;
    typedef boost::shared_ptr <DifferentiableFunction>
    DifferentiableFunctionPtr_t;
    typedef boost::shared_ptr <DiffusingPlanner> DiffusingPlannerPtr_t;
    typedef boost::shared_ptr <DiscretizedCollisionChecking>
    DiscretizedCollisionCheckingPtr_t;
    typedef boost::shared_ptr <Distance> DistancePtr_t;
    typedef Edge* EdgePtr_t;
    typedef std::list <Edge*> Edges_t;
    typedef boost::shared_ptr <ExtractedPath> ExtractedPathPtr_t;
    typedef model::JointJacobian_t JointJacobian_t;
    typedef model::Joint Joint;
    typedef model::JointPtr_t JointPtr_t;
    typedef boost::shared_ptr <JointBoundValidation> JointBoundValidationPtr_t;
    typedef model::HalfJointJacobian_t HalfJointJacobian_t;
    typedef model::JointVector_t JointVector_t;
    typedef boost::shared_ptr <LockedDof> LockedDofPtr_t;
    typedef model::matrix_t matrix_t;
    typedef Eigen::Ref <const matrix_t> matrixIn_t;
    typedef Eigen::Ref <matrix_t> matrixOut_t;
    typedef std::list <Node*> Nodes_t;
    typedef Node* NodePtr_t;
    typedef model::ObjectVector_t ObjectVector_t;
    typedef boost::shared_ptr <Path> PathPtr_t;
    typedef boost::shared_ptr <PathOptimizer> PathOptimizerPtr_t;
    typedef boost::shared_ptr <PathPlanner> PathPlannerPtr_t;
    typedef boost::shared_ptr <PathValidation> PathValidationPtr_t;
    typedef boost::shared_ptr <PathVector> PathVectorPtr_t;
    typedef boost::shared_ptr <PlanAndOptimize> PlanAndOptimizePtr_t;
    typedef Problem* ProblemPtr_t;
    typedef ProblemSolver* ProblemSolverPtr_t;
    typedef boost::shared_ptr <RandomShortcut> RandomShortcutPtr_t;
    typedef boost::shared_ptr <Roadmap> RoadmapPtr_t;
    typedef boost::shared_ptr <StraightPath> StraightPathPtr_t;
    typedef boost::shared_ptr <SteeringMethod> SteeringMethodPtr_t;
    typedef boost::shared_ptr <SteeringMethodStraight>
    SteeringMethodStraightPtr_t;
    typedef roboptim::StableTimePoint StableTimePoint_t;
    typedef std::vector <PathPtr_t> Paths_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef model::vector_t vector_t;
    typedef model::vectorIn_t vectorIn_t;
    typedef model::vectorOut_t vectorOut_t;
    typedef boost::shared_ptr <WeighedDistance> WeighedDistancePtr_t;
    typedef KDTree* KDTreePtr_t;
    typedef std::map <std::string, DifferentiableFunctionPtr_t>
    DifferentiableFunctionMap_t;
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_FWD_HH
