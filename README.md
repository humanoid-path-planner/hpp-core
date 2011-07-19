hppCore implements basic classes used as interfaces with KineoPathPlanner.
The main classes are:

    * hpp::core::Device: implement a humanoid robot with a dynamical model.
    * hpp::core::Problem: define a canonical path planning problem.
    * hpp::core::Planner: contains a vector of above path planning problems to
      implement iterative planning algorithms that use several instaciations of
      a robot.

This package and depending packages implementing algorithms can be used
with or without GUI, depending on whether we are developing and debugging
new algorithms or we want to run the algorithms on-board a robot.

Upon some events (a problem is added in hpp::core::Planner object, a path has
been planned, ...), notifications are sent.

    * When working with the GUI, these notification are caught by the
      interface and objects are added in the view,
    * when working without interface, the notification have no effet.
