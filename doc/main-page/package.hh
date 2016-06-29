/** \mainpage
    \section hpp_core_sec_intro Introduction

    This package implements path planning algorithms for robots as
    kinematic chains.  Kinematic chains are represented by class
    hpp::model::Device in package hpp-model.

    The main classes are:
    \li hpp::core::Problem: defines a canonical path planning problem,
    \li hpp::core::PathPlanner: implements an algorithm to solve a problem,
    \li hpp::core::Roadmap: stores a network of collision-free paths
    \li hpp::core::SteeringMethod: builds paths between configurations taking
    into account kinematic constraints.
    \li hpp::core::Path: Abstraction of \ref path for a robot.

    For clarity, classes are distributed into <a
    href="modules.html">modules</a>.

    \section hpp_core_sec_embedding Embedding hpp-core into an application

    Class hpp::core::ProblemSolver is a container aiming at embedding
    hpp-core into an application. It stores elements of the problem
    that can be provided in random order and builds a valid problem
    upon call of method solve. After completion of method solve, it
    stores solution paths.

    \defgroup configuration_sampling Configuration Sampling

    Random sampling of robot configurations for random path planning.

    \defgroup path_planning Path planning algorithms

    Path planning algorithms derive from class hpp::core::PathPlanner.

    \defgroup path_optimization Path Optimization

    Path optimization algorithms derive from class hpp::core::PathOptimizer

    \defgroup steering_method Steering method and distance functions

    Some system are subject to kinematic or dynamic constraints. Those
    constraints can be handled in path planning using a steering method
    that builds an admissible path between two configurations of the system.
    When using a steering method, it can be useful to use a distance function
    that accounts for the cost to go from a configuration to another.

    \defgroup validation Validation of configurations and paths

    Paths and configurations need to be validated with respect to various
    criteria (collision, joint bounds,...) during path planning and path
    optimization. Validation of a configuration or of a path gives rise to
    a validation report that provide information if validation failed.

    \defgroup roadmap Roadmap

    Random sampling algorithm build a representation of the free configuration
    space as a graph called a roadmap. Nodes are configurations (or states) and
    edges are collision-free admissible paths (or trajectories).

    \defgroup constraints Constraints

    Some robots can be subject to constraints.
    \li closed kinematic chains,
    \li quasi-static equilibrium for legged robots

    are examples of such constraints. These constraints can be defined
    implicitely by an equation the left hand side of which is a
    differentiable function of the robot configuration.

    Classes of this module enable users to define such constraints by
    deriving class Constraint and by storing instances in
    ConstraintSet.

    ConfigProjector defines implicit constraints by
    equalities and/or inequalities between a \link DifferentiableFunction
    differentiable function \endlink and a constant vector.

    LockedJoint set the position of one joint to a fixed value.

    Each ConstraintSet may contain at most one ConfigProjector. LockedJoint
    instances added to a ConstraintSet are in fact stored by the ConfigProjector
    of the ConstraintSet if any. The rationale is that the computations that
    solve numerical constraints are optimized by solving only for non locked
    degrees of freedom.

    Below is a diagram of the classes that implement constraints with the
    description of some methods to help advanced user to understand how those
    classes interact between each other.

    \htmlonly
   <table cellspacing="20">
      <tbody><tr>
          <td>  <h4>Constraint
    	</h4>
          </td>
          <td><h4>ConstraintSet</h4>
          </td>
        </tr>
        <tr>
          <td>    addToSet (set):<br>
    	&nbsp set.push_back (self)<br><br>
    	addLockedJoint (lockedDof):<br>
    	&nbsp pass</td>
          <td>addToSet (set):<br>
    	&nbsp for c in constraints_:<br>
    	&nbsp &nbsp c.addToSet (set)<br><br>
    	addLockedJoint (ld):<br>
    	&nbsp for c in constraints_:<br>
    	&nbsp &nbsp c.addLockedJoint (ld) <br><br>
    	addConstraint (c):<br>
    	&nbsp c.addToSet (self)</td>
        </tr>
        <tr>		<td><h4>ConfigProjector
    	</h4>
    	addToSet (set):<br>
    	&nbsp if set.configProjector_:<br>
    	&nbsp &nbsp throw <br>
    	&nbsp if set.lockedDof_:<br>
    	&nbsp &nbsp throw <br>
    	&nbsp set.removeConfigProjector ()<br>
    	&nbsp set.configProjector_ = self<br>
    	&nbsp Constraint::addToSet (set)</td>
          <td><h4>LockedJoint</h4>
    	addToSet (set):<br>
    	&nbsp set.lockedJoint_ = true <br>
    	&nbsp set.addLockedJoint (self)</td>
        </tr>
      </tbody>
    </table>
    \endhtmlonly
*/
