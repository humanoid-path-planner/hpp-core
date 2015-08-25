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
# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (BasicConfigurationShooter);
    HPP_PREDEF_CLASS (CollisionPathValidation);
    struct CollisionPathValidationReport;
    HPP_PREDEF_CLASS (CollisionValidation);
    HPP_PREDEF_CLASS (CollisionValidationReport);
    HPP_PREDEF_CLASS (ConfigurationShooter);
    HPP_PREDEF_CLASS (ConfigProjector);
    HPP_PREDEF_CLASS (ConfigValidation);
    HPP_PREDEF_CLASS (ConfigValidations);
    HPP_PREDEF_CLASS (ConnectedComponent);
    HPP_PREDEF_CLASS (Constraint);
    HPP_PREDEF_CLASS (ConstraintSet);
    HPP_PREDEF_CLASS (DiffusingPlanner);
    HPP_PREDEF_CLASS (Distance);
    HPP_PREDEF_CLASS (DistanceBetweenObjects);
    HPP_PREDEF_CLASS (DiscretizedCollisionChecking);
    HPP_PREDEF_CLASS (NumericalConstraint);
    HPP_PREDEF_CLASS (LockedJoint);
    class Edge;
    HPP_PREDEF_CLASS (ExtractedPath);
    HPP_PREDEF_CLASS (JointBoundValidation);
    struct JointBoundValidationReport;
    class Node;
    HPP_PREDEF_CLASS (Path);
    HPP_PREDEF_CLASS (PathOptimizer);
    HPP_PREDEF_CLASS (PathPlanner);
    HPP_PREDEF_CLASS (PathVector);
    HPP_PREDEF_CLASS (PathValidation);
    struct PathValidationReport;
    HPP_PREDEF_CLASS (PathValidation);
    HPP_PREDEF_CLASS (PlanAndOptimize);
    HPP_PREDEF_CLASS (Problem);
    class ProblemSolver;
    HPP_PREDEF_CLASS (RandomShortcut);
    HPP_PREDEF_CLASS (Roadmap);
    HPP_PREDEF_CLASS (SteeringMethod);
    HPP_PREDEF_CLASS (SteeringMethodStraight);
    HPP_PREDEF_CLASS (StraightPath);
    HPP_PREDEF_CLASS (ValidationReport);
    HPP_PREDEF_CLASS (VisibilityPrmPlanner);
    HPP_PREDEF_CLASS (WeighedDistance);
    class KDTree;

    class ComparisonType;
    typedef boost::shared_ptr <ComparisonType> ComparisonTypePtr_t;
    class Equality;
    typedef boost::shared_ptr <Equality> EqualityPtr_t;
    class EqualToZero;
    typedef boost::shared_ptr <EqualToZero> EqualToZeroPtr_t;
    class ComparisonTypes;
    typedef boost::shared_ptr <ComparisonTypes> ComparisonTypesPtr_t;
    class DoubleInequality;
    typedef boost::shared_ptr <DoubleInequality> DoubleInequalityPtr_t;

    typedef boost::shared_ptr < BasicConfigurationShooter >
    BasicConfigurationShooterPtr_t;
    typedef hpp::model::Body Body;
    typedef hpp::model::BodyPtr_t BodyPtr_t;
    typedef boost::shared_ptr <CollisionPathValidationReport>
    CollisionPathValidationReportPtr_t;
    typedef std::vector <CollisionPathValidationReport> 
    CollisionPathValidationReports_t;
    typedef boost::shared_ptr <CollisionValidation> CollisionValidationPtr_t;
    typedef boost::shared_ptr <CollisionValidationReport>
    CollisionValidationReportPtr_t;
    typedef model::CollisionObjectPtr_t CollisionObjectPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef model::ConfigurationPtr_t ConfigurationPtr_t;
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
    typedef boost::shared_ptr <const ConstraintSet> ConstraintSetConstPtr_t;
    typedef model::Device Device_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::DeviceWkPtr_t DeviceWkPtr_t;
    typedef model::CenterOfMassComputationPtr_t CenterOfMassComputationPtr_t;
    typedef std::deque <DevicePtr_t> Devices_t;
    typedef constraints::DifferentiableFunction DifferentiableFunction;
    typedef constraints::DifferentiableFunctionPtr_t
    DifferentiableFunctionPtr_t;
    typedef boost::shared_ptr <DiffusingPlanner> DiffusingPlannerPtr_t;
    typedef boost::shared_ptr <DiscretizedCollisionChecking>
    DiscretizedCollisionCheckingPtr_t;
    typedef boost::shared_ptr <Distance> DistancePtr_t;
    typedef boost::shared_ptr <DistanceBetweenObjects>
    DistanceBetweenObjectsPtr_t;
    typedef model::DistanceResult DistanceResult;
    typedef model::DistanceResults_t DistanceResults_t;
    typedef Edge* EdgePtr_t;
    typedef std::list <Edge*> Edges_t;
    typedef boost::shared_ptr <ExtractedPath> ExtractedPathPtr_t;
    typedef model::JointJacobian_t JointJacobian_t;
    typedef model::Joint Joint;
    typedef model::JointConstPtr_t JointConstPtr_t;
    typedef model::JointPtr_t JointPtr_t;
    typedef boost::shared_ptr <JointBoundValidation> JointBoundValidationPtr_t;
    typedef boost::shared_ptr <JointBoundValidationReport>
    JointBoundValidationReportPtr_t;
    typedef model::HalfJointJacobian_t HalfJointJacobian_t;
    typedef model::JointVector_t JointVector_t;
    typedef KDTree* KDTreePtr_t;
    typedef boost::shared_ptr <LockedJoint> LockedJointPtr_t;
    typedef boost::shared_ptr <const LockedJoint> LockedJointConstPtr_t;
    typedef boost::shared_ptr <NumericalConstraint> NumericalConstraintPtr_t;
    typedef model::matrix_t matrix_t;
    typedef constraints::matrixIn_t matrixIn_t;
    typedef constraints::matrixOut_t matrixOut_t;
    typedef model::size_type size_type;
    typedef model::value_type value_type;
    typedef std::pair<value_type, value_type> interval_t;
    typedef std::pair<size_type, size_type> SizeInterval_t;
    typedef std::vector < SizeInterval_t > SizeIntervals_t;
    typedef std::list <Node*> Nodes_t;
    typedef Node* NodePtr_t;
    typedef model::ObjectVector_t ObjectVector_t;
    typedef boost::shared_ptr <Path> PathPtr_t;
    typedef boost::shared_ptr <const Path> PathConstPtr_t;
    typedef boost::shared_ptr <PathOptimizer> PathOptimizerPtr_t;
    typedef boost::shared_ptr <PathPlanner> PathPlannerPtr_t;
    typedef boost::shared_ptr <PathValidation> PathValidationPtr_t;
    typedef boost::shared_ptr <PathValidationReport> PathValidationReportPtr_t;
    typedef boost::shared_ptr <PathVector> PathVectorPtr_t;
    typedef boost::shared_ptr <const PathVector> PathVectorConstPtr_t;
    typedef boost::shared_ptr <PlanAndOptimize> PlanAndOptimizePtr_t;
    typedef Problem* ProblemPtr_t;
    typedef ProblemSolver* ProblemSolverPtr_t;
    typedef boost::shared_ptr <RandomShortcut> RandomShortcutPtr_t;
    typedef boost::shared_ptr <Roadmap> RoadmapPtr_t;
    typedef boost::shared_ptr <StraightPath> StraightPathPtr_t;
    typedef boost::shared_ptr <const StraightPath> StraightPathConstPtr_t;
    typedef boost::shared_ptr <SteeringMethod> SteeringMethodPtr_t;
    typedef boost::shared_ptr <SteeringMethodStraight>
    SteeringMethodStraightPtr_t;
    typedef std::vector <PathPtr_t> Paths_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef model::vector3_t vector3_t;
    typedef model::vector_t vector_t;
    typedef model::vectorIn_t vectorIn_t;
    typedef model::vectorOut_t vectorOut_t;
    typedef Eigen::Matrix<value_type, 1, Eigen::Dynamic> rowvector_t;
    typedef boost::shared_ptr <VisibilityPrmPlanner> VisibilityPrmPlannerPtr_t;
    typedef boost::shared_ptr <ValidationReport> ValidationReportPtr_t;
    typedef boost::shared_ptr <WeighedDistance> WeighedDistancePtr_t;
    typedef std::map <std::string, DifferentiableFunctionPtr_t>
    DifferentiableFunctionMap_t;
    typedef std::map <std::string, ComparisonTypePtr_t>
    ComparisonTypeMap_t;
    typedef std::map <std::string, SizeIntervals_t> SizeIntervalsMap_t;
    typedef std::vector < NumericalConstraintPtr_t > NumericalConstraints_t;
    typedef std::map <std::string, CenterOfMassComputationPtr_t>
    CenterOfMassComputationMap_t;

    // Collision pairs
    typedef std::pair <CollisionObjectPtr_t, CollisionObjectPtr_t>
    CollisionPair_t;
    typedef std::list <CollisionPair_t> CollisionPairs_t;

    namespace continuousCollisionChecking {
      HPP_PREDEF_CLASS (Dichotomy);
      typedef boost::shared_ptr <Dichotomy> DichotomyPtr_t;
      HPP_PREDEF_CLASS (Progressive);
      typedef boost::shared_ptr <Progressive> ProgressivePtr_t;
    } // namespace continuousCollisionChecking

    class NearestNeighbor;
    typedef NearestNeighbor* NearestNeighborPtr_t;
    namespace nearestNeighbor {
      class Basic;
      class KDTree;
      typedef KDTree* KDTreePtr_t;
      typedef Basic* BasicPtr_t;
    } // namespace nearestNeighbor

    namespace pathOptimization {
      HPP_PREDEF_CLASS (Cost);
      typedef boost::shared_ptr <Cost> CostPtr_t;
      HPP_PREDEF_CLASS (GradientBased);
      typedef boost::shared_ptr <GradientBased> GradientBasedPtr_t;
      HPP_PREDEF_CLASS (PathLength);
      typedef boost::shared_ptr <PathLength> PathLengthPtr_t;
    } // namespace pathOptimization

    HPP_PREDEF_CLASS (PathProjector);
    typedef boost::shared_ptr <PathProjector> PathProjectorPtr_t;
    namespace pathProjector {
      HPP_PREDEF_CLASS (Dichotomy);
      typedef boost::shared_ptr <Dichotomy> DichotomyPtr_t;
      HPP_PREDEF_CLASS (Progressive);
      typedef boost::shared_ptr <Progressive> ProgressivePtr_t;
    } // namespace pathProjector
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_FWD_HH
