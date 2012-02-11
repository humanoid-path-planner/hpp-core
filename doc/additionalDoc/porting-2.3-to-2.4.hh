/*!
\page porting Porting code from version 2.3 to 2.4
This page explains how to port client code from version 2.3 to 2.4.
\section hpp_core_porting_sec_goal_conf Goal configurations

From version 2.4 on, several goal configurations can be defined by users.
In some motion planning problems, this feature is usefull. For instance,
in manipulation planning problems, the goal is defined as a submanifold
of the configuration space. In this case, it is efficient to sample the
goal sub-manifold and to add goal configurations to the roadmap builder.

The following methods have changed or are new in 2.4

\subsection hpp_core_porting_subsec_planner hpp::core::Planner class

\li hpp::core::Planner::initConfIthProblem(unsigned int rank, const CkwsConfigShPtr& config) asserts that preconditions are satisfied and returns void.
\li hpp::core::Planner::goalConfIthProblem(unsigned int rank) const now return a vector
of shared pointers to goal configurations instead of a single shared
pointer.
\li hpp::core::Planner::goalConfIthProblem(unsigned int rank, const CkwsConfigShPtr config) has been replaced by hpp::core::Planner::addGoalConfIthProblem(unsigned int rank, const CkwsConfigShPtr& config). This new method returns void.
\li hpp::core::Planner::void resetGoalConfIthProblem (unsigned int rank) has been
added.

\subsection hpp_core_porting_subsec_problem hpp::core::Problem class

\li hpp::core::Problem::initConfig (const CkwsConfigShPtr& inConfig) asserts that
preconditions are satisfied and returns void.
\li hpp::core::Problem::goalConfig (const CkwsConfigShPtr& inConfig) has been replaced
by hpp::core::Problem::addGoalConfig (const CkwsConfigShPtr& inConfig). This latter
method returns void.
\li hpp::core::Problem::resetGoalConfig () has been added.
*/
