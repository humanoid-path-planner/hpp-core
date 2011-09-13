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

SET(${PROJECT_NAME}_HEADERS
  hpp/core/fwd.hh
  hpp/core/planner.hh
  hpp/core/problem.hh
  hpp/core/collision-pair.hh
  )
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/hpp/core
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )

IF(${ENABLE_BODY} STREQUAL YES)
  SET(${PROJECT_NAME}_HEADERS hpp-model/hpp/model/body.hh)
  INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
    DESTINATION include/hpp/model
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    )
ELSE(${ENABLE_BODY} STREQUAL YES)
  SET(${PROJECT_NAME}_HEADERS 
    hpp/core/parser.hh
    hpp/core/fwd.hh
    )
  INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
    DESTINATION include/hpp/core
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    )
  
ENDIF(${ENABLE_BODY} STREQUAL YES)
