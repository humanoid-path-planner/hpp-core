# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

- [core] Cap the QP solver's iterations in SplineGradientBased
- [Path] Fix time-parameterized extraction and reversal
- [SplineGradientBased] Split interpolated paths before smoothing
- ROS: example-robot-{data -> descriptions} - #454
- [InterpolatedPath] Bound velocity over every overlapping interval
- Bound two previously-unbounded loops in planning/validation by wall-clock timeout - #450
- Fix continuous collision validation when a freeflyer is grasped
- [path-planning] Fix planner hook documentation - #447

## [9.0.2] - 2026-07-24



## [9.0.0] - 2026-07-13



## [8.0.0] - 2026-07-06

(yes, this release was forgotten)

## [7.0.0] - 2026-03-06



## [6.1.0] - 2025-10-23



## [6.0.0] - 2024-12-07

Changes in v6.0.0
- hpp-fcl dependency has been replaced by coal
- updates for coal v3
- add SearchInRoadmap path planner
- [doc] add path planner algorightms in path planner module


## [5.2.0] - 2024-10-09

Changes in v5.2.0:
- fix wrong assert
- plugins: look for HPP_PLUGIN_DIRS
- nix: move package to nixpkgs
- ci: use https
- setup mergify


## [5.1.0] - 2024-07-02

Changes in v5.1.0:
- remove usage of deprecated symbols
- update for pinocchio 3
- Nix: initial support
- update tooling


## [5.0.0] - 2024-03-31

Changes in v5.0.0:
- :warning: BREAKING: proxsuite is now a required dependency
- Remove deprecated symbols
- Remove use of ConfigurationPtr_t
- Use proxqp instead of eiquadprog for solving QP programs
- update for hpp-fcl v3
- make Path::timeParameterization getter public
- Fix spline steering method when path length != 1
- Improve PiecewisePolynomial
- Clarification of the tests in PathPlanner::solve()
- Improve performances of continuous path validation
- [PathPlanner] Add getters for maxIterations and timeOut
- Add seedable uniform configuration shooter
- Pass computation flag to Device::computeForwardKinematics
- Fix computation of valid interval in BodyPairCollision
- Make SplineGradientBased more configurable
- Remove log output and relax accuracy in test
- update packaging
- update tooling
- bug fixes

## [4.15.1] - 2023-01-20



## [4.14.0] - 2022-11-02



## [4.13.0] - 2022-05-31

to fix build with hpp-fcl v2


## [4.12.0] - 2021-10-06

Changes in v4.12.0:
- make ConstantCurvature::Wheels_t public for C++17
- Fix handling of obstacles in Problem
- Fix compilation warnings in tests.
- [PathPlanner] throw specific type of exception when failing.
- [Roadmap] Add method to insert a PathVector in a roadmap.
- [SimpleTimeParameterization] Fix method optimize.
- implementation of velocity bounds for path vector.

## [4.11.0] - 2021-05-04

Changes in v4.11.0:
- Serialize several classes for save and loading roadmaps
- use shared pointers to class Problem only. Classes previously storing
  const references to Problem now store const weak pointers. Classes
  that use to take a const reference to problem in the constructor now
  take a const shared pointer.
- Make Path::operator() deprecated: use method Path::eval instead.
- Improve Vmax computation for solid-solid collision
- [ConfigProjector] Add method isSatisfied with input error threshold.

## [4.10.1] - 2020-09-24

Changes since v4.9.0:
* Add a parameter to set the center of the Gaussiant configuration shooter.
* Fix bug in Reeds and Shepp paths when curvature is not equal to 1.
* Handle LockedJoint and Implicit instances in the same container in ProblemSolver class. make method ProblemSolver::addLockedJointToConfigProjector deprecated.
* Replace class ContinuousCollisionChecking by class ContinuousValidation for broader generalisation.
* Remove deprecated methods and files
  - ProblemSolver::addLockedJoint,
  - hpp::core::JointBoundException,
  - include/hpp/core/continuous-collision-checking.hh,
  - include/hpp/core/continuous-collision-checking/dichotomy.hh,
  - include/hpp/core/continuous-collision-checking/progressive.hh,
  - include/hpp/core/discretized-collision-checking.hh,
  - include/hpp/core/discretized-path-validation.hh,
  - include/hpp/core/locked-joint.hh,
  - include/hpp/core/random-shortcut.hh,
  - include/hpp/core/steering-method-straight.hh.
* in SteeringMethod, assert that if q1 == q2, operator() returns a path.
* class SplineGradientBased has been improved
  - some bugs have been fixed,
  - length of input path should not be 0.
* PathVector::extract(t,t) does not returns an empty path.
* InterpolatedPath supports non-zero start of interval of definition
* Add abstract class ObstacleUserInterface. This class is aimed at being parent of an class handling obstacles. Provide some implementations like
  - ObstacleUser,
  - ObstacleUserVector.
* Add method setSecurityMargin to class ObstacleUserInterface.

## [4.10.0] - 2020-08-17



## [4.9.1] - 2020-07-05



## [4.9.0] - 2020-04-29

Changes in v4.9.0:
- Fix bug in Reeds and Shepp paths when curvature is not equal to 1.
- Handle LockedJoint and Implicit instances in the same container in ProblemSolver class. make method ProblemSolver::addLockedJointToConfigProjector deprecated.
- Replace class ContinuousCollisionChecking by class ContinuousValidation for broader generalisation.
- Remove deprecated methods and files
  - ProblemSolver::addLockedJoint,
  - hpp::core::JointBoundException,
  - include/hpp/core/continuous-collision-checking.hh,
  - include/hpp/core/continuous-collision-checking/dichotomy.hh,
  - include/hpp/core/continuous-collision-checking/progressive.hh,
  - include/hpp/core/discretized-collision-checking.hh,
  - include/hpp/core/discretized-path-validation.hh,
  - include/hpp/core/locked-joint.hh,
  - include/hpp/core/random-shortcut.hh,
  - include/hpp/core/steering-method-straight.hh.
- in SteeringMethod, assert that if q1 == q2, operator() returns a path.
- class SplineGradientBased has been improved
  - some bugs have been fixed,
  - length of input path should not be 0.
- PathVector::extract(t,t) does not returns an empty path.
- InterpolatedPath supports non-zero start of interval of definition
- Add abstract class ObstacleUserInterface. This class is aimed at being parent of an class handling obstacles. Provide some implementations like
  - ObstacleUser,
  - ObstacleUserVector.
- Add method setSecurityMargin to class ObstacleUserInterface.
- CMake Exports


## [4.8.0] - 2019-11-28


Changes since v4.7.0:
- fix build with gcc 9 / -std=c++11
- Add getter and setter for the line search type of ConfigProjector.
- Improve some error messages.
- Move steeringMethod::Straight::impl_compute to definition file.
- Update CMake

## [4.7.0] - 2019-10-04

Changes since v4.6.0:
- [ProblemSolver] Remove deprecated method.
- Remove old files + Fix usage of JointModelComposite.
- Minor changes and fixes for continuous validation
- [ReedsSheppPath] Fix bug when curvature is different from 1.


## [4.5.0] - 2019-04-24

Changes since v4.4.0:
- [PathPlanner] Replace tryDirectPath by tryConnectInitAndGoals
- [SimpleShortcut] Implement new path optimizer.
- [CMake] fix GPL lib dependencies
- Enhance carlike paths.


## [4.4.0] - 2019-03-19

Changes since v4.3.0:
- Update to Pinocchio v2 + plugins + varying right hand side.
- Fix error in steering-kinodynamic due to numerical imprecision
- [BUG FIX] fixing initialization of collision requests in body pair co�
- [CMake] install pdfs only if required
- conforming to new api for collisionrequest


## [4.3.0] - 2019-01-31

- Reorganize path validation + fix SplineGradientBased
- fix build with recent boost version


## [4.2.0] - 2018-10-11

Changes since v4.1:
- Refactor constraints
- Update to modification in class constraints::Explicit:
  - constructors and create methods take a LiegroupSpace as input instead of a DevicePtr_t.
- Update to modifications in hpp::constraints::ExplicitConstraintSet:
  - freeDers -> notOutDers,
  - derSize -> nv,
  - viewJacobian -> jacobianNotOutToOut.
- Migrate some methods and members from ConfigProjector to BySubstitution


## [4.0] - 2018-01-03

From this version on, hpp now depends on pinocchio package for all computations of forwad kinematics.

## [3.2] - 2017-03-17

[Unreleased]: https://github.com/humanoid-path-planner/hpp-core/compare/v9.0.2...HEAD
[9.0.2]: https://github.com/humanoid-path-planner/hpp-core/compare/v9.0.0...v9.0.2
[9.0.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v8.0.0...v9.0.0
[8.0.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v7.0.0...v8.0.0
[7.0.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v6.1.0...v7.0.0
[6.1.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v6.0.0...v6.1.0
[6.0.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v5.2.0...v6.0.0
[5.2.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v5.1.0...v5.2.0
[5.1.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v5.0.0...v5.1.0
[5.0.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.15.1...v5.0.0
[4.15.1]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.14.0...v4.15.1
[4.14.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.13.0...v4.14.0
[4.13.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.12.0...v4.13.0
[4.12.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.11.0...v4.12.0
[4.11.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.10.1...v4.11.0
[4.10.1]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.10.0...v4.10.1
[4.10.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.9.1...v4.10.0
[4.9.1]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.9.0...v4.9.1
[4.9.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.8.0...v4.9.0
[4.8.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.7.0...v4.8.0
[4.7.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.5.0...v4.7.0
[4.5.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.4.0...v4.5.0
[4.4.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.3.0...v4.4.0
[4.3.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.2.0...v4.3.0
[4.2.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v4.0...v4.2.0
[4.0]: https://github.com/humanoid-path-planner/hpp-core/compare/v3.2...v4.0
[3.2]: https://github.com/humanoid-path-planner/hpp-core/releases/tag/v3.2
