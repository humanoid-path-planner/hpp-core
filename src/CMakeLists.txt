#
# Copyright (c) 2010, 2011 CNRS
# Authors: Florent Lamiraux
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
  collision-validation.cc
  config-projector.cc
  comparison-type.cc
  config-validations.cc
  connected-component.cc
  constraint.cc
  constraint-set.cc
  continuous-collision-checking/dichotomy.cc
  continuous-collision-checking/dichotomy/body-pair-collision.hh
  continuous-collision-checking/progressive.cc
  continuous-collision-checking/progressive/body-pair-collision.hh
  diffusing-planner.cc
  discretized-collision-checking.cc
  distance-between-objects.cc
  extracted-path.hh
  joint-bound-validation.cc
  nearest-neighbor/basic.hh
  nearest-neighbor/k-d-tree.cc
  nearest-neighbor/k-d-tree.hh
  node.cc
  path.cc
  path-optimization/collision-constraints-result.hh
  path-optimization/path-length.cc
  path-optimization/gradient-based.cc
  path-planner.cc
  path-vector.cc
  plan-and-optimize.cc
  problem.cc
  problem-solver.cc
  random-shortcut.cc
  roadmap.cc
  straight-path.cc
  visibility-prm-planner.cc
  weighed-distance.cc
  numerical-constraint.cc
  equation.cc
  locked-joint.cc
  path-projector/progressive.cc
  path-projector/dichotomy.cc
  path-projector.cc
  parser/roadmap-factory.cc
  )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constraints)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
