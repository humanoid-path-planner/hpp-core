//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_CORE_FWD_HH
#define HPP_CORE_FWD_HH

#include <deque>
#include <hpp/constraints/fwd.hh>
#include <hpp/core/config.hh>
#include <hpp/core/deprecated.hh>
#include <hpp/util/pointer.hh>
#include <list>
#include <map>
#include <set>
#include <vector>

namespace hpp {
namespace core {
HPP_PREDEF_CLASS(BiRRTPlanner);
HPP_PREDEF_CLASS(CollisionValidation);
HPP_PREDEF_CLASS(CollisionValidationReport);
HPP_PREDEF_CLASS(AllCollisionsValidationReport);
HPP_PREDEF_CLASS(ConfigurationShooter);
HPP_PREDEF_CLASS(ConfigProjector);
HPP_PREDEF_CLASS(ConfigValidation);
HPP_PREDEF_CLASS(ConfigValidations);
HPP_PREDEF_CLASS(ConnectedComponent);
HPP_PREDEF_CLASS(Constraint);
HPP_PREDEF_CLASS(ConstraintSet);
HPP_PREDEF_CLASS(DiffusingPlanner);
HPP_PREDEF_CLASS(Distance);
HPP_PREDEF_CLASS(DistanceBetweenObjects);
class Edge;
HPP_PREDEF_CLASS(ExtractedPath);
HPP_PREDEF_CLASS(SubchainPath);
HPP_PREDEF_CLASS(JointBoundValidation);
struct JointBoundValidationReport;
class Node;
HPP_PREDEF_CLASS(Path);
HPP_PREDEF_CLASS(TimeParameterization);
HPP_PREDEF_CLASS(PathOptimizer);
HPP_PREDEF_CLASS(PathPlanner);
HPP_PREDEF_CLASS(ProblemTarget);
HPP_PREDEF_CLASS(PathVector);
HPP_PREDEF_CLASS(PlanAndOptimize);
HPP_PREDEF_CLASS(Problem);
class ProblemSolver;
HPP_PREDEF_CLASS(Roadmap);
HPP_PREDEF_CLASS(SteeringMethod);
HPP_PREDEF_CLASS(StraightPath);
HPP_PREDEF_CLASS(InterpolatedPath);
HPP_PREDEF_CLASS(DubinsPath);
HPP_PREDEF_CLASS(ReedsSheppPath);
HPP_PREDEF_CLASS(KinodynamicPath);
HPP_PREDEF_CLASS(KinodynamicOrientedPath);
HPP_PREDEF_CLASS(ValidationReport);
HPP_PREDEF_CLASS(VisibilityPrmPlanner);
HPP_PREDEF_CLASS(WeighedDistance);
HPP_PREDEF_CLASS(KinodynamicDistance);
class KDTree;

typedef constraints::ComparisonTypes_t ComparisonTypes_t;
typedef constraints::ComparisonType ComparisonType;

typedef shared_ptr<BiRRTPlanner> BiRRTPlannerPtr_t;
typedef hpp::pinocchio::Body Body;
typedef hpp::pinocchio::BodyPtr_t BodyPtr_t;
typedef shared_ptr<CollisionValidation> CollisionValidationPtr_t;
typedef shared_ptr<CollisionValidationReport> CollisionValidationReportPtr_t;
typedef shared_ptr<AllCollisionsValidationReport>
    AllCollisionsValidationReportPtr_t;
typedef pinocchio::CollisionObjectPtr_t CollisionObjectPtr_t;
typedef pinocchio::CollisionObjectConstPtr_t CollisionObjectConstPtr_t;
typedef pinocchio::FclCollisionObject FclCollisionObject;
typedef FclCollisionObject* FclCollisionObjectPtr_t;
typedef const FclCollisionObject* FclConstCollisionObjectPtr_t;
typedef shared_ptr<FclCollisionObject> FclCollisionObjectSharePtr_t;

typedef pinocchio::Configuration_t Configuration_t;
typedef pinocchio::ConfigurationIn_t ConfigurationIn_t;
typedef pinocchio::ConfigurationOut_t ConfigurationOut_t;
typedef pinocchio::ConfigurationPtr_t ConfigurationPtr_t;
typedef std::vector<ConfigurationPtr_t> Configurations_t;
typedef Configurations_t::iterator ConfigIterator_t;
typedef Configurations_t::const_iterator ConfigConstIterator_t;
typedef shared_ptr<ConfigurationShooter> ConfigurationShooterPtr_t;
typedef shared_ptr<ConfigProjector> ConfigProjectorPtr_t;
typedef shared_ptr<ConfigValidation> ConfigValidationPtr_t;
typedef shared_ptr<ConfigValidations> ConfigValidationsPtr_t;
typedef shared_ptr<ConnectedComponent> ConnectedComponentPtr_t;

struct SharedComparator {
    template <typename T>
    bool operator()(const std::shared_ptr<T>& lhs, const std::shared_ptr<T>& rhs) const {
        return (*lhs) < (*rhs);
    }
};
typedef std::set<ConnectedComponentPtr_t, SharedComparator> ConnectedComponents_t;

typedef shared_ptr<Constraint> ConstraintPtr_t;
typedef shared_ptr<ConstraintSet> ConstraintSetPtr_t;
typedef shared_ptr<const ConstraintSet> ConstraintSetConstPtr_t;
typedef std::vector<ConstraintPtr_t> Constraints_t;
typedef pinocchio::Device Device_t;
typedef pinocchio::DevicePtr_t DevicePtr_t;
typedef pinocchio::DeviceWkPtr_t DeviceWkPtr_t;
typedef pinocchio::CenterOfMassComputationPtr_t CenterOfMassComputationPtr_t;
typedef std::deque<DevicePtr_t> Devices_t;
typedef constraints::DifferentiableFunction DifferentiableFunction;
typedef constraints::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
typedef shared_ptr<DiffusingPlanner> DiffusingPlannerPtr_t;
typedef shared_ptr<Distance> DistancePtr_t;
typedef shared_ptr<DistanceBetweenObjects> DistanceBetweenObjectsPtr_t;
typedef pinocchio::DistanceResults_t DistanceResults_t;
typedef Edge* EdgePtr_t;
typedef std::list<Edge*> Edges_t;
typedef shared_ptr<ExtractedPath> ExtractedPathPtr_t;
typedef shared_ptr<SubchainPath> SubchainPathPtr_t;
typedef pinocchio::JointJacobian_t JointJacobian_t;
typedef pinocchio::Joint Joint;
typedef pinocchio::JointConstPtr_t JointConstPtr_t;
typedef pinocchio::JointPtr_t JointPtr_t;
typedef pinocchio::JointConstPtr_t JointConstPtr_t;
typedef shared_ptr<JointBoundValidation> JointBoundValidationPtr_t;
typedef shared_ptr<JointBoundValidationReport> JointBoundValidationReportPtr_t;
typedef pinocchio::HalfJointJacobian_t HalfJointJacobian_t;
typedef pinocchio::JointVector_t JointVector_t;
typedef KDTree* KDTreePtr_t;
typedef constraints::LockedJoint LockedJoint;
typedef constraints::LockedJointPtr_t LockedJointPtr_t;
typedef constraints::LockedJointConstPtr_t LockedJointConstPtr_t;
typedef constraints::LockedJoints_t LockedJoints_t;
typedef pinocchio::matrix_t matrix_t;
typedef pinocchio::matrix3_t matrix3_t;
typedef constraints::matrix6_t matrix6_t;
typedef pinocchio::vector3_t vector3_t;
typedef constraints::matrixIn_t matrixIn_t;
typedef constraints::matrixOut_t matrixOut_t;
typedef constraints::LiegroupElement LiegroupElement;
typedef constraints::LiegroupElementRef LiegroupElementRef;
typedef pinocchio::LiegroupElementConstRef LiegroupElementConstRef;
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
typedef std::list<NodePtr_t> Nodes_t;
typedef std::vector<NodePtr_t> NodeVector_t;
typedef pinocchio::ObjectVector_t ObjectVector_t;
typedef std::vector<CollisionObjectPtr_t> ObjectStdVector_t;
typedef std::vector<CollisionObjectConstPtr_t> ConstObjectStdVector_t;

typedef shared_ptr<Path> PathPtr_t;
typedef shared_ptr<const Path> PathConstPtr_t;
typedef shared_ptr<TimeParameterization> TimeParameterizationPtr_t;
typedef shared_ptr<PathOptimizer> PathOptimizerPtr_t;
typedef shared_ptr<PathPlanner> PathPlannerPtr_t;
typedef shared_ptr<ProblemTarget> ProblemTargetPtr_t;
typedef shared_ptr<PathVector> PathVectorPtr_t;
typedef shared_ptr<const PathVector> PathVectorConstPtr_t;
typedef shared_ptr<PlanAndOptimize> PlanAndOptimizePtr_t;
typedef shared_ptr<Problem> ProblemPtr_t;
typedef shared_ptr<const Problem> ProblemConstPtr_t;
typedef ProblemSolver* ProblemSolverPtr_t;
typedef shared_ptr<Roadmap> RoadmapPtr_t;
typedef shared_ptr<StraightPath> StraightPathPtr_t;
typedef shared_ptr<const StraightPath> StraightPathConstPtr_t;
typedef shared_ptr<ReedsSheppPath> ReedsSheppPathPtr_t;
typedef shared_ptr<const ReedsSheppPath> ReedsSheppPathConstPtr_t;
typedef shared_ptr<DubinsPath> DubinsPathPtr_t;
typedef shared_ptr<const DubinsPath> DubinsPathConstPtr_t;
typedef shared_ptr<KinodynamicPath> KinodynamicPathPtr_t;
typedef shared_ptr<const KinodynamicPath> KinodynamicPathConstPtr_t;
typedef shared_ptr<KinodynamicOrientedPath> KinodynamicOrientedPathPtr_t;
typedef shared_ptr<const KinodynamicOrientedPath>
    KinodynamicOrientedPathConstPtr_t;
typedef shared_ptr<InterpolatedPath> InterpolatedPathPtr_t;
typedef shared_ptr<const InterpolatedPath> InterpolatedPathConstPtr_t;
typedef shared_ptr<SteeringMethod> SteeringMethodPtr_t;
typedef std::vector<PathPtr_t> Paths_t;
typedef std::vector<PathVectorPtr_t> PathVectors_t;
typedef std::vector<PathVectorPtr_t> PathVectors_t;
typedef pinocchio::Transform3f Transform3f;
typedef pinocchio::vector3_t vector3_t;
typedef Eigen::Matrix<value_type, 2, 1> vector2_t;
typedef pinocchio::vector_t vector_t;
typedef pinocchio::vectorIn_t vectorIn_t;
typedef pinocchio::vectorOut_t vectorOut_t;
typedef Eigen::Matrix<value_type, 1, Eigen::Dynamic> rowvector_t;
typedef shared_ptr<VisibilityPrmPlanner> VisibilityPrmPlannerPtr_t;
typedef shared_ptr<ValidationReport> ValidationReportPtr_t;
typedef shared_ptr<WeighedDistance> WeighedDistancePtr_t;
typedef shared_ptr<KinodynamicDistance> KinodynamicDistancePtr_t;
typedef std::map<std::string, constraints::ImplicitPtr_t>
    NumericalConstraintMap_t;
typedef std::map<std::string, ComparisonTypes_t> ComparisonTypeMap_t;
typedef std::map<std::string, segments_t> segmentsMap_t;
typedef constraints::NumericalConstraints_t NumericalConstraints_t;
typedef std::map<std::string, CenterOfMassComputationPtr_t>
    CenterOfMassComputationMap_t;

// Collision pairs
struct CollisionPair;
typedef CollisionPair CollisionPair_t;  // For backward compatibility.
typedef std::vector<CollisionPair_t> CollisionPairs_t;

class ExtractedPath;
namespace path {
template <int _PolynomeBasis, int _Order>
class Spline;
HPP_PREDEF_CLASS(Hermite);
typedef shared_ptr<Hermite> HermitePtr_t;
typedef shared_ptr<const Hermite> HermiteConstPtr_t;
}  // namespace path

HPP_PREDEF_CLASS(ContinuousValidation);
typedef shared_ptr<ContinuousValidation> ContinuousValidationPtr_t;
namespace continuousValidation {
HPP_PREDEF_CLASS(Dichotomy);
typedef shared_ptr<Dichotomy> DichotomyPtr_t;
HPP_PREDEF_CLASS(Progressive);
typedef shared_ptr<Progressive> ProgressivePtr_t;
HPP_PREDEF_CLASS(BodyPairCollision);
typedef shared_ptr<BodyPairCollision> BodyPairCollisionPtr_t;
typedef std::vector<BodyPairCollisionPtr_t> BodyPairCollisions_t;
HPP_PREDEF_CLASS(IntervalValidation);
typedef shared_ptr<IntervalValidation> IntervalValidationPtr_t;
typedef std::vector<IntervalValidationPtr_t> IntervalValidations_t;
HPP_PREDEF_CLASS(SolidSolidCollision);
typedef shared_ptr<SolidSolidCollision> SolidSolidCollisionPtr_t;
}  // namespace continuousValidation

namespace distance {
HPP_PREDEF_CLASS(ReedsShepp);
typedef shared_ptr<ReedsShepp> ReedsSheppPtr_t;
}  // namespace distance

class NearestNeighbor;
typedef NearestNeighbor* NearestNeighborPtr_t;
namespace nearestNeighbor {
class Basic;
class KDTree;
typedef KDTree* KDTreePtr_t;
typedef Basic* BasicPtr_t;
}  // namespace nearestNeighbor

namespace pathOptimization {
HPP_PREDEF_CLASS(RandomShortcut);
typedef shared_ptr<RandomShortcut> RandomShortcutPtr_t;
HPP_PREDEF_CLASS(SimpleShortcut);
typedef shared_ptr<SimpleShortcut> SimpleShortcutPtr_t;
HPP_PREDEF_CLASS(Cost);
typedef shared_ptr<Cost> CostPtr_t;
HPP_PREDEF_CLASS(GradientBased);
typedef shared_ptr<GradientBased> GradientBasedPtr_t;
HPP_PREDEF_CLASS(PathLength);
typedef shared_ptr<PathLength> PathLengthPtr_t;
HPP_PREDEF_CLASS(PartialShortcut);
typedef shared_ptr<PartialShortcut> PartialShortcutPtr_t;
HPP_PREDEF_CLASS(SimpleTimeParameterization);
typedef shared_ptr<SimpleTimeParameterization> SimpleTimeParameterizationPtr_t;
HPP_PREDEF_CLASS(ConfigOptimization);
typedef shared_ptr<ConfigOptimization> ConfigOptimizationPtr_t;
}  // namespace pathOptimization

namespace pathPlanner {
HPP_PREDEF_CLASS(kPrmStar);
typedef shared_ptr<kPrmStar> kPrmStarPtr_t;
}  // namespace pathPlanner

HPP_PREDEF_CLASS(PathValidations);
HPP_PREDEF_CLASS(PathValidation);
typedef shared_ptr<PathValidation> PathValidationPtr_t;
typedef shared_ptr<PathValidations> PathValidationsPtr_t;
namespace pathValidation {
HPP_PREDEF_CLASS(Discretized);
typedef shared_ptr<Discretized> DiscretizedPtr_t;
}  // namespace pathValidation
// Path validation reports
struct PathValidationReport;
struct CollisionPathValidationReport;
typedef shared_ptr<PathValidationReport> PathValidationReportPtr_t;
typedef shared_ptr<CollisionPathValidationReport>
    CollisionPathValidationReportPtr_t;
typedef std::vector<CollisionPathValidationReport>
    CollisionPathValidationReports_t;

HPP_PREDEF_CLASS(PathProjector);
typedef shared_ptr<PathProjector> PathProjectorPtr_t;
namespace pathProjector {
HPP_PREDEF_CLASS(Global);
typedef shared_ptr<Global> GlobalPtr_t;
HPP_PREDEF_CLASS(Dichotomy);
typedef shared_ptr<Dichotomy> DichotomyPtr_t;
HPP_PREDEF_CLASS(Progressive);
typedef shared_ptr<Progressive> ProgressivePtr_t;
HPP_PREDEF_CLASS(RecursiveHermite);
typedef shared_ptr<RecursiveHermite> RecursiveHermitePtr_t;
}  // namespace pathProjector

namespace steeringMethod {
HPP_PREDEF_CLASS(Interpolated);
typedef shared_ptr<Interpolated> InterpolatedPtr_t;
HPP_PREDEF_CLASS(ReedsShepp);
typedef shared_ptr<ReedsShepp> ReedsSheppPtr_t;
HPP_PREDEF_CLASS(Kinodynamic);
typedef shared_ptr<Kinodynamic> KinodynamicPtr_t;
}  // namespace steeringMethod

namespace problemTarget {
HPP_PREDEF_CLASS(GoalConfigurations);
HPP_PREDEF_CLASS(TaskTarget);
typedef shared_ptr<GoalConfigurations> GoalConfigurationsPtr_t;
typedef shared_ptr<TaskTarget> TaskTargetPtr_t;
}  // namespace problemTarget

namespace configurationShooter {
HPP_PREDEF_CLASS(Uniform);
typedef shared_ptr<Uniform> UniformPtr_t;
HPP_PREDEF_CLASS(Gaussian);
typedef shared_ptr<Gaussian> GaussianPtr_t;
}  // namespace configurationShooter

/// Plane polygon represented by its vertices
/// Used to model contact surfaces for manipulation applications
typedef constraints::Shape_t Shape_t;
typedef constraints::JointAndShape_t JointAndShape_t;
typedef constraints::JointAndShapes_t JointAndShapes_t;

namespace continuousCollisionChecking = continuousValidation;

}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_FWD_HH
