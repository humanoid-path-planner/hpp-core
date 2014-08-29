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
  config-validations.cc
  connected-component.cc
  constraint.cc
  constraint-set.cc
  diffusing-planner.cc
  discretized-collision-checking.cc
  extracted-path.hh
  joint-bound-validation.cc
  nearest-neighbor.hh
  node.cc
  path.cc
  path-planner.cc
  path-vector.cc
  plan-and-optimize.cc
  problem.cc
  problem-solver.cc
  random-shortcut.cc
  roadmap.cc
  straight-path.cc
  weighed-distance.cc
  k-d-tree.cc
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} roboptim-trajectory)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
