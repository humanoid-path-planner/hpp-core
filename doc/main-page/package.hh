/// \mainpage
/// \section hpp_core_sec_intro Introduction
///
/// This package implements path planning algorithms for robots as
/// kinematic chains.  Kinematic chains are represented by class
/// hpp::model::Device in package hpp-model.
///
/// The main classes are:
/// \li hpp::core::Problem: defines a canonical path planning problem,
/// \li hpp::core::PathPlanner: implements an algorithm to solve a problem,
/// \li hpp::core::Roadmap: stores a network of collision-free paths
/// \li hpp::core::SteeringMethod: builds paths between configurations taking
/// into account kinematic constraints.
/// \li hpp::core::Path: paths for a robot.
///
/// For clarity, classes are distributed into <a
/// href="modules.html">modules</a>.
///
/// \section hpp_core_sec_embedding Embedding hpp-core into an application
///
/// Class hpp::core::ProblemSolver is a container aiming at embedding
/// hpp-core into an application. It stores elements of the problem
/// that can be provided in random order and builds a valid problem
/// upon call of method solve. After completion of method solve, it
/// stores solution paths.
///
/// \defgroup configuration_sampling Configuration Sampling
///
/// Random sampling of robot configurations for random path planning.
///
/// \defgroup path_planning Path planning algorithms
///
/// Path planning algorithms derive from class hpp::core::PathPlanner.
///
/// \defgroup path_optimization Path Optimization
///
/// Path optimization algorithms derive from class hpp::core::PathOptimizer
///
/// \defgroup steering_method Steering method and distance functions
///
/// Some system are subject to kinematic or dynamic constraints. Those
/// constraints can be handled in path planning using a steering method
/// that builds an admissible path between two configurations of the system.
/// When using a steering method, it can be useful to use a distance function
/// that accounts for the cost to go from a configuration to another.
///
/// \defgroup validation Validation of configurations and paths
///
/// Paths and configurations need to be validated with respect to various
/// criteria (collision, joint bounds,...) during path planning and path
/// optimization. Validation of a configuration or of a path gives rise to
/// a validation report that provide information if validation failed. 
///
/// \defgroup roadmap Roadmap
///
/// Random sampling algorithm build a representation of the free configuration
/// space as a graph called a roadmap. Nodes are configurations (or states) and
/// edges are collision-free admissible paths (or trajectories).
///
