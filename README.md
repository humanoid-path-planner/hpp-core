This package implements path planning algorithms for kinematic chains.
Kinematic chains are represented by class hpp::model::Device.

The main classes are:

    * hpp::core::Problem: defines a canonical path planning problem,
    * hpp::core::PathPlanner: implements an algorithm to solve a problem,
    * hpp::core::Roadmap: stores a network of collision-free paths
    * hpp::core::SteeringMethod: builds paths between configurations taking into
      account kinematic constraints.
    * hpp::core::Path: paths for a robot.

Embedding hpp-core into an application

Class hpp::core::ProblemSolver is a container aiming at embedding
hpp-core into an application. It stores elements of the problem
that can be provided in random order and builds a valid problem
upon call of method solve. After completion of method solve, it
stores solution paths.
