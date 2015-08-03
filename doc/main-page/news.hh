/**

\page hpp_core_news News

New features from one version to another.

\section hpp_core_news_3_1to_3_2 New features between version 3.1 and 3.2

\li Several path optimizer can be inserted in the problem solver. When calling method \c solve, they are called in the order in which they have been inserted. Method \c clearPathOptimizers has also been added.

\li Copy constructors have been implemented in classes related to constraints. Copying a path now implies to copy the associated constraints. This modification fixes some hazards in the use of constraints.

\li A static method that creates a \c ProblemSolver instance has been added.
    Using this method makes the latest created instance available by static 
    method \c ProblemSolver::latest.

\li ConfigProjector::computeValueAndJacobian is now public. This enables users
    to handle themselves a set of numerical constraints, and to implement their
    own resolution algorithm. In the same class, method \c compressVector,
    \c uncompressVector, \c compressMatrix, \c uncompressMatrix enable users
    to switch from velocity vectors without locked degrees of freedom to
    velocity vector that contains locked degrees of freedom, and to switch
    from matrices without the columns (and the lines for Hessian matrices)
    corresponding to locked degrees of freedom to matrices that contain them.
