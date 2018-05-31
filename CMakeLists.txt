#
# Copyright (c) 2010, 2011 CNRS
# Authors: Florent Lamiraux, Joseph Mirabel
#
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME hpp-core)
SET(PROJECT_DESCRIPTION
  "Implement basic classes for canonical path planning for kinematic chains."
  )
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()
# Activate hpp-util logging if requested
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmark output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()
# Ask Doxygen to create a tree view in html documentation
SET(DOXYGEN_TREEVIEW "NO" CACHE STRING "Set to YES to generate a tree view in the html documentation")

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
  include/hpp/core/basic-configuration-shooter.hh # DEPRECATED
  include/hpp/core/bi-rrt-planner.hh
  include/hpp/core/collision-path-validation-report.hh
  include/hpp/core/collision-validation.hh
  include/hpp/core/relative-motion.hh
  include/hpp/core/collision-validation-report.hh
  include/hpp/core/projection-error.hh
  include/hpp/core/configuration-shooter.hh
  include/hpp/core/configuration-shooter/uniform.hh
  include/hpp/core/configuration-shooter/gaussian.hh
  include/hpp/core/config-projector.hh
  include/hpp/core/config-validation.hh
  include/hpp/core/config-validations.hh
  include/hpp/core/connected-component.hh
  include/hpp/core/constraint.hh
  include/hpp/core/constraint-set.hh
  include/hpp/core/container.hh
  include/hpp/core/continuous-collision-checking.hh
  include/hpp/core/continuous-collision-checking/dichotomy.hh
  include/hpp/core/continuous-collision-checking/progressive.hh
  include/hpp/core/diffusing-planner.hh
  include/hpp/core/discretized-collision-checking.hh
  include/hpp/core/discretized-path-validation.hh
  include/hpp/core/distance/reeds-shepp.hh
  include/hpp/core/distance.hh
  include/hpp/core/distance-between-objects.hh
  include/hpp/core/dubins-path.hh
  include/hpp/core/edge.hh
  include/hpp/core/explicit-numerical-constraint.hh
  include/hpp/core/explicit-relative-transformation.hh
  include/hpp/core/fwd.hh
  include/hpp/core/joint-bound-validation.hh
  include/hpp/core/equation.hh
  include/hpp/core/path-validations.hh
  include/hpp/core/numerical-constraint.hh
  include/hpp/core/locked-joint.hh
  include/hpp/core/node.hh
  include/hpp/core/parameter.hh
  include/hpp/core/path.hh
  include/hpp/core/path-optimization/path-length.hh
  include/hpp/core/path-optimization/gradient-based.hh
  include/hpp/core/path-optimization/linear-constraint.hh
  include/hpp/core/path-optimization/partial-shortcut.hh
  include/hpp/core/path-optimization/quadratic-program.hh
  include/hpp/core/path-optimization/config-optimization.hh
  include/hpp/core/path-optimization/simple-time-parameterization.hh
  include/hpp/core/path-optimization/spline-gradient-based.hh
  include/hpp/core/path-optimization/spline-gradient-based-abstract.hh
  include/hpp/core/path-optimizer.hh
  include/hpp/core/path-planner.hh
  include/hpp/core/path-planner/k-prm-star.hh
  include/hpp/core/path-validation.hh
  include/hpp/core/path-validation-report.hh
  include/hpp/core/path-vector.hh
  include/hpp/core/path/spline.hh
  include/hpp/core/path/hermite.hh
  include/hpp/core/reeds-shepp-path.hh
  include/hpp/core/plan-and-optimize.hh
  include/hpp/core/problem.hh
  include/hpp/core/problem-solver.hh
  include/hpp/core/random-shortcut.hh
  include/hpp/core/roadmap.hh
  include/hpp/core/steering-method.hh
  include/hpp/core/steering-method/fwd.hh
  include/hpp/core/steering-method/straight.hh
  include/hpp/core/steering-method/car-like.hh
  include/hpp/core/steering-method/constant-curvature.hh
  include/hpp/core/steering-method/dubins.hh
  include/hpp/core/steering-method/hermite.hh
  include/hpp/core/steering-method/reeds-shepp.hh
  include/hpp/core/steering-method/spline.hh
  include/hpp/core/steering-method/snibud.hh
  include/hpp/core/steering-method-straight.hh
  include/hpp/core/straight-path.hh
  include/hpp/core/interpolated-path.hh
  include/hpp/core/validation-report.hh
  include/hpp/core/visibility-prm-planner.hh
  include/hpp/core/weighed-distance.hh
  include/hpp/core/path-projector/progressive.hh
  include/hpp/core/path-projector/dichotomy.hh
  include/hpp/core/path-projector/global.hh
  include/hpp/core/path-projector/recursive-hermite.hh
  include/hpp/core/path-projector.hh
  include/hpp/core/nearest-neighbor.hh
  include/hpp/core/parser/roadmap-factory.hh
  include/hpp/core/problem-target.hh
  include/hpp/core/problem-target/goal-configurations.hh
  include/hpp/core/problem-target/task-target.hh
  include/hpp/core/subchain-path.hh
  include/hpp/core/time-parameterization.hh
  include/hpp/core/time-parameterization/polynomial.hh
  )

ADD_REQUIRED_DEPENDENCY("hpp-pinocchio >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-statistics >= 1.1")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4")

ADD_SUBDIRECTORY(src)
SET(BOOST_COMPONENTS system timer unit_test_framework)
SEARCH_FOR_BOOST()
ADD_SUBDIRECTORY(tests)

# Add dependency toward hpp-core library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-core")

INSTALL (FILES doc/continuous-collision-checking.pdf
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html)

SETUP_PROJECT_FINALIZE()
