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
  collision-pair.cc
  planner.cc
  problem.cc
)

IF (ENABLE_BODY)
  SET(${LIBRARY_NAME}_SOURCES
    ${${LIBRARY_NAME}_SOURCES}
    body.cc
    )
  INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR}/include/hpp-model)
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${LIBRARY_NAME}_SOURCES
)

  SET(${LIBRARY_NAME}_SOURCE_FILES
    ${${LIBRARY_NAME}_SOURCE_FILES}
    body.cc)
ELSE(${ENABLE_BODY} STREQUAL YES)
  # Add hppModel compilation flags and link to library libhppModel.so
  INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR}/include)
  ADD_DEFINITIONS(${HPPMODEL_CFLAGS})
ENDIF()
