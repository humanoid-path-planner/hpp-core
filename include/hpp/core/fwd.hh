//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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
# include <map>
# include <deque>
# include <list>
# include <set>
# include <hpp/util/pointer.hh>
# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace core {
    HPP_PREDEF_CLASS (BasicConfigurationShooter);
    HPP_PREDEF_CLASS (BiRRTPlanner);
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
    HPP_PREDEF_CLASS (DiscretizedPathValidation);
    HPP_PREDEF_CLASS (PathValidations);
    HPP_PREDEF_CLASS (Equation);
    HPP_PREDEF_CLASS (ExplicitNumericalConstraint);
    HPP_PREDEF_CLASS (ExplicitRelativeTransformation);
    HPP_PREDEF_CLASS (NumericalConstraint);
    HPP_PREDEF_CLASS (LockedJoint);
    class Edge;
    HPP_PREDEF_CLASS (ExtractedPath);
    HPP_PREDEF_CLASS (SubchainPath);
    HPP_PREDEF_CLASS (JointBoundValidation);
    struct JointBoundValidationReport;
    class Node;
    HPP_PREDEF_CLASS (Path);
    HPP_PREDEF_CLASS (TimeParameterization);
    HPP_PREDEF_CLASS (PathOptimizer);
    HPP_PREDEF_CLASS (PathPlanner);
    HPP_PREDEF_CLASS (ProblemTarget);
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
    HPP_PREDEF_CLASS (StraightPath);
    HPP_PREDEF_CLASS (InterpolatedPath);
    HPP_PREDEF_CLASS (DubinsPath);
    HPP_PREDEF_CLASS (ReedsSheppPath);
    HPP_PREDEF_CLASS (ValidationReport);
    HPP_PREDEF_CLASS (VisibilityPrmPlanner);
    HPP_PREDEF_CLASS (WeighedDistance);
    class KDTree;

    typedef constraints::ComparisonTypes_t ComparisonTypes_t;
    typedef constraints::ComparisonType ComparisonType;

    typedef boost::shared_ptr < BasicConfigurationShooter >
    BasicConfigurationShooterPtr_t;
    typedef boost::shared_ptr <BiRRTPlanner> BiRRTPlannerPtr_t;
    typedef hpp::pinocchio::Body Body;
    typedef hpp::pinocchio::BodyPtr_t BodyPtr_t;
    typedef boost::shared_ptr <CollisionPathValidationReport>
    CollisionPathValidationReportPtr_t;
    typedef std::vector <CollisionPathValidationReport> 
    CollisionPathValidationReports_t;
    typedef boost::shared_ptr <CollisionValidation> CollisionValidationPtr_t;
    typedef boost::shared_ptr <CollisionValidationReport>
    CollisionValidationReportPtr_t;
    typedef pinocchio::CollisionObjectPtr_t CollisionObjectPtr_t;
    typedef pinocchio::CollisionObjectConstPtr_t CollisionObjectConstPtr_t;
    typedef pinocchio::FclCollisionObject FclCollisionObject;

    typedef pinocchio::Configuration_t Configuration_t;
    typedef pinocchio::ConfigurationIn_t ConfigurationIn_t;
    typedef pinocchio::ConfigurationOut_t ConfigurationOut_t;
    typedef pinocchio::ConfigurationPtr_t ConfigurationPtr_t;
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
    typedef std::deque <ConstraintPtr_t> Constraints_t;
    typedef pinocchio::Device Device_t;
    typedef pinocchio::DevicePtr_t DevicePtr_t;
    typedef pinocchio::DeviceWkPtr_t DeviceWkPtr_t;
    typedef pinocchio::CenterOfMassComputationPtr_t CenterOfMassComputationPtr_t;
    typedef std::deque <DevicePtr_t> Devices_t;
    typedef constraints::DifferentiableFunction DifferentiableFunction;
    typedef constraints::DifferentiableFunctionPtr_t
    DifferentiableFunctionPtr_t;
    typedef boost::shared_ptr <DiffusingPlanner> DiffusingPlannerPtr_t;
    typedef boost::shared_ptr <DiscretizedCollisionChecking>
    DiscretizedCollisionCheckingPtr_t;
    typedef boost::shared_ptr <DiscretizedPathValidation>
    DiscretizedPathValidationPtr_t;
    typedef boost::shared_ptr <PathValidations>
    PathValidationsPtr_t;
    typedef boost::shared_ptr <Distance> DistancePtr_t;
    typedef boost::shared_ptr <DistanceBetweenObjects>
    DistanceBetweenObjectsPtr_t;
    typedef pinocchio::DistanceResults_t DistanceResults_t;
    typedef Edge* EdgePtr_t;
    typedef std::list <Edge*> Edges_t;
    typedef boost::shared_ptr <ExplicitNumericalConstraint>
    ExplicitNumericalConstraintPtr_t;
    typedef boost::shared_ptr <ExplicitRelativeTransformation>
    ExplicitRelativeTransformationPtr_t;
    typedef boost::shared_ptr <ExtractedPath> ExtractedPathPtr_t;
    typedef boost::shared_ptr <SubchainPath> SubchainPathPtr_t;
    typedef pinocchio::JointJacobian_t JointJacobian_t;
    typedef pinocchio::Joint Joint;
    typedef pinocchio::JointConstPtr_t JointConstPtr_t;
    typedef pinocchio::JointPtr_t JointPtr_t;
    typedef pinocchio::JointConstPtr_t JointConstPtr_t;
    typedef boost::shared_ptr <JointBoundValidation> JointBoundValidationPtr_t;
    typedef boost::shared_ptr <JointBoundValidationReport>
    JointBoundValidationReportPtr_t;
    typedef pinocchio::HalfJointJacobian_t HalfJointJacobian_t;
    typedef pinocchio::JointVector_t JointVector_t;
    typedef KDTree* KDTreePtr_t;
    typedef boost::shared_ptr <LockedJoint> LockedJointPtr_t;
    typedef boost::shared_ptr <Equation> EquationPtr_t;
    typedef boost::shared_ptr <const LockedJoint> LockedJointConstPtr_t;
    typedef boost::shared_ptr <NumericalConstraint> NumericalConstraintPtr_t;
    typedef std::list <LockedJointPtr_t> LockedJoints_t;
    typedef pinocchio::matrix_t matrix_t;
    typedef pinocchio::matrix3_t matrix3_t;
    typedef constraints::matrix6_t matrix6_t;
    typedef pinocchio::vector3_t vector3_t;
    typedef constraints::matrixIn_t matrixIn_t;
    typedef constraints::matrixOut_t matrixOut_t;
    typedef constraints::LiegroupElement LiegroupElement;
    typedef constraints::LiegroupSpace LiegroupSpace;
    typedef constraints::LiegroupSpacePtr_t LiegroupSpacePtr_t;
    typedef pinocchio::size_type size_type;
    typedef pinocchio::value_type value_type;
    typedef std::pair<value_type, value_type> interval_t;
    /// Interval of indices as (first index, number of indices)
    typedef Eigen::BlockIndex BlockIndex;
    typedef constraints::segment_t segment_t;
    typedef constraints::segments_t segments_t;
    typedef Node* NodePtr_t;
    typedef std::list <NodePtr_t> Nodes_t;
    typedef std::vector <NodePtr_t> NodeVector_t;
    typedef pinocchio::ObjectVector_t ObjectVector_t;
    typedef std::vector<CollisionObjectPtr_t> ObjectStdVector_t;
    typedef std::vector<CollisionObjectConstPtr_t> ConstObjectStdVector_t;

    typedef boost::shared_ptr <Path> PathPtr_t;
    typedef boost::shared_ptr <const Path> PathConstPtr_t;
    typedef boost::shared_ptr <TimeParameterization> TimeParameterizationPtr_t;
    typedef boost::shared_ptr <PathOptimizer> PathOptimizerPtr_t;
    typedef boost::shared_ptr <PathPlanner> PathPlannerPtr_t;
    typedef boost::shared_ptr <ProblemTarget> ProblemTargetPtr_t;
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
    typedef boost::shared_ptr <ReedsSheppPath> ReedsSheppPathPtr_t;
    typedef boost::shared_ptr <const ReedsSheppPath> ReedsSheppPathConstPtr_t;
    typedef boost::shared_ptr <DubinsPath> DubinsPathPtr_t;
    typedef boost::shared_ptr <const DubinsPath> DubinsPathConstPtr_t;
    typedef boost::shared_ptr <InterpolatedPath> InterpolatedPathPtr_t;
    typedef boost::shared_ptr <const InterpolatedPath> InterpolatedPathConstPtr_t;
    typedef boost::shared_ptr <SteeringMethod> SteeringMethodPtr_t;
    typedef std::vector <PathPtr_t> Paths_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef std::vector <PathVectorPtr_t> PathVectors_t;
    typedef pinocchio::Transform3f Transform3f;
    typedef pinocchio::vector3_t vector3_t;
    typedef Eigen::Matrix<value_type, 2, 1> vector2_t;
    typedef pinocchio::vector_t vector_t;
    typedef pinocchio::vectorIn_t vectorIn_t;
    typedef pinocchio::vectorOut_t vectorOut_t;
    typedef Eigen::Matrix<value_type, 1, Eigen::Dynamic> rowvector_t;
    typedef boost::shared_ptr <VisibilityPrmPlanner> VisibilityPrmPlannerPtr_t;
    typedef boost::shared_ptr <ValidationReport> ValidationReportPtr_t;
    typedef boost::shared_ptr <WeighedDistance> WeighedDistancePtr_t;
    typedef std::map <std::string, NumericalConstraintPtr_t>
    NumericalConstraintMap_t;
    typedef std::map <std::string, ComparisonTypes_t> ComparisonTypeMap_t;
    typedef std::map <std::string, segments_t> segmentsMap_t;
    typedef std::vector < NumericalConstraintPtr_t > NumericalConstraints_t;
    typedef std::map <std::string, CenterOfMassComputationPtr_t>
    CenterOfMassComputationMap_t;

    // Collision pairs
    typedef std::pair <CollisionObjectConstPtr_t, CollisionObjectConstPtr_t>
    CollisionPair_t;
    typedef std::vector <CollisionPair_t> CollisionPairs_t;

    class ExtractedPath;
    namespace path {
      template <int _PolynomeBasis, int _Order> class Spline;
      HPP_PREDEF_CLASS (Hermite);
      typedef boost::shared_ptr <Hermite> HermitePtr_t;
      typedef boost::shared_ptr <const Hermite> HermiteConstPtr_t;
    } // namespace path

    HPP_PREDEF_CLASS (ContinuousCollisionChecking);
    typedef boost::shared_ptr <ContinuousCollisionChecking>
    ContinuousCollisionCheckingPtr_t;
    namespace continuousCollisionChecking {
      HPP_PREDEF_CLASS (Dichotomy);
      typedef boost::shared_ptr <Dichotomy> DichotomyPtr_t;
      HPP_PREDEF_CLASS (Progressive);
      typedef boost::shared_ptr <Progressive> ProgressivePtr_t;
    } // namespace continuousCollisionChecking

    namespace distance {
      HPP_PREDEF_CLASS (ReedsShepp);
      typedef boost::shared_ptr <ReedsShepp> ReedsSheppPtr_t;
    } // namespace distance

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
      HPP_PREDEF_CLASS (PartialShortcut);
      typedef boost::shared_ptr <PartialShortcut> PartialShortcutPtr_t;
      HPP_PREDEF_CLASS (SimpleTimeParameterization);
      typedef boost::shared_ptr <SimpleTimeParameterization> SimpleTimeParameterizationPtr_t;
      HPP_PREDEF_CLASS (ConfigOptimization);
      typedef boost::shared_ptr <ConfigOptimization>
        ConfigOptimizationPtr_t;
    } // namespace pathOptimization

    namespace pathPlanner {
      HPP_PREDEF_CLASS (kPrmStar);
      typedef boost::shared_ptr <kPrmStar> kPrmStarPtr_t;
    } // namespace pathPlanner

    HPP_PREDEF_CLASS (PathProjector);
    typedef boost::shared_ptr <PathProjector> PathProjectorPtr_t;
    namespace pathProjector {
      HPP_PREDEF_CLASS (Global);
      typedef boost::shared_ptr <Global> GlobalPtr_t;
      HPP_PREDEF_CLASS (Dichotomy);
      typedef boost::shared_ptr <Dichotomy> DichotomyPtr_t;
      HPP_PREDEF_CLASS (Progressive);
      typedef boost::shared_ptr <Progressive> ProgressivePtr_t;
      HPP_PREDEF_CLASS (RecursiveHermite);
      typedef boost::shared_ptr <RecursiveHermite> RecursiveHermitePtr_t;
    } // namespace pathProjector

    namespace problemTarget {
      HPP_PREDEF_CLASS (GoalConfigurations);
      HPP_PREDEF_CLASS (TaskTarget);
      typedef boost::shared_ptr <GoalConfigurations> GoalConfigurationsPtr_t;
      typedef boost::shared_ptr <TaskTarget> TaskTargetPtr_t;
    } // namespace problemTarget

    typedef std::vector<core::vector3_t> Shape_t;
    typedef std::pair <JointPtr_t, Shape_t> JointAndShape_t;
    typedef std::list <JointAndShape_t> JointAndShapes_t;
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_FWD_HH
