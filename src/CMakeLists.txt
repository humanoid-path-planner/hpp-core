#
# Copyright (c) 2010, 2011, 2012, 2013, 2014, 2015, 2016 CNRS
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

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
astar.hh
bi-rrt-planner.cc
collision-validation.cc
configuration-shooter/uniform.cc
configuration-shooter/gaussian.cc
config-projector.cc
config-validations.cc
connected-component.cc
constraint.cc
constraint-set.cc
continuous-validation.cc
continuous-validation/initializer.cc
continuous-validation/body-pair-collision.cc
continuous-validation/dichotomy.cc
continuous-validation/solid-solid-collision.cc
continuous-validation/progressive.cc
diffusing-planner.cc
distance/reeds-shepp.cc
distance-between-objects.cc
  dubins.hh
  dubins.cc
  dubins-path.cc
extracted-path.hh
interpolated-path.cc
joint-bound-validation.cc
path-validations.cc
path-validation/discretized.cc
path-validation/discretized-collision-checking.cc
path-validation/discretized-joint-bound.cc
  nearest-neighbor/basic.hh #
  nearest-neighbor/basic.cc #
  # nearest-neighbor/k-d-tree.cc #
  # nearest-neighbor/k-d-tree.hh #
  node.cc #
  parameter.cc #
  path.cc #
  path-optimizer.cc #
  path-optimization/linear-constraint.cc #
  path-optimization/spline-gradient-based-abstract.cc #
  path-optimization/partial-shortcut.cc #
  path-optimization/random-shortcut.cc
  path-optimization/simple-time-parameterization.cc#
  path-planner.cc #
  path-planner/k-prm-star.cc
  path-vector.cc #
  path/spline.cc
  path/hermite.cc
  plan-and-optimize.cc #
  plugin.cc # TODO : addObstacle
  problem.cc # TODO : addObstacle
  problem-solver.cc # TODO : addObstacle
  path-projector.cc
  path-projector/dichotomy.cc #
  path-projector/global.cc #
  path-projector/progressive.cc #
  path-projector/recursive-hermite.cc
  path-projector.cc #
  parser/roadmap-factory.cc #
  problem-target/goal-configurations.cc #
  problem-target/task-target.cc #
  reeds-shepp-path.cc
  relative-motion.cc  
  kinodynamic-path.cc
  kinodynamic-oriented-path.cc
  steering-method/steering-kinodynamic.cc
  roadmap.cc
  steering-method/reeds-shepp.cc # TODO access type of joint
  steering-method/car-like.cc
  steering-method/constant-curvature.cc
  steering-method/dubins.cc
  steering-method/snibud.cc
  steering-method/spline.cc
  straight-path.cc
  interpolated-path.cc
  visibility-prm-planner.cc
  weighed-distance.cc
  kinodynamic-distance.cc
  path-projector/progressive.cc
  path-projector/dichotomy.cc
  path-projector/global.cc
  path-projector.cc
  parser/roadmap-factory.cc
  problem-target/goal-configurations.cc
  problem-target/task-target.cc
  )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constraints)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

# GPL licenced part
ADD_LIBRARY(${LIBRARY_NAME}-gpl
  SHARED
  path-optimization/quadratic-program.cc
  path-optimization/spline-gradient-based.cc)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME}-gpl hpp-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME}-gpl hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME}-gpl hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME}-gpl hpp-constraints)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME}-gpl ${LIBRARY_NAME})

INSTALL(TARGETS ${LIBRARY_NAME}-gpl DESTINATION lib)
