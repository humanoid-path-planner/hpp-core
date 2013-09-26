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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-core)
SET(PROJECT_DESCRIPTION
  "Implement basic classes used as interfaces with KineoPathPlanner"
  )
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SETUP_PROJECT()

# Activate hpp-util logging if requested
IF (HPP_DEBUG)
  MESSAGE(STATUS "Activate hpp-util logging.")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
  include/hpp/core/fwd.hh
  include/hpp/core/collision-pair.hh
  include/hpp/core/planner.hh
  include/hpp/core/problem.hh
)

ADD_REQUIRED_DEPENDENCY("hpp-kwsplus >= 2.3")
ADD_REQUIRED_DEPENDENCY("hpp-model >= 2.1")

# Add dependency toward hpp-model library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-core")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
