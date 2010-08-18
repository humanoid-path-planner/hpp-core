# 
#  Copyright 2010 CNRS-LAAS
#  Author: Florent Lamiraux
# 

#----------------------------------------------------
# Install old style deprecated header files
#----------------------------------------------------

SET(${PROJECT_NAME}_HEADERS_OLD_STYLE
  hppCore/hppColPair.h
  hppCore/hppPlanner.h
  hppCore/hppProblem.h
)

INSTALL(FILES ${${PROJECT_NAME}_HEADERS_OLD_STYLE}
  DESTINATION include/${PROJECT_NAME}
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

#----------------------------------------------------
# Install new style header files
#----------------------------------------------------

SET(${PROJECT_NAME}_HEADERS 
  hpp/core/planner.hh
  hpp/core/problem.hh
  hpp/core/collision-pair.hh
  )
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/hpp/core
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )

IF(${ENABLE_BODY} STREQUAL YES)
  SET(${PROJECT_NAME}_HEADERS hppModel/hppModel/hppBody.h)
  INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
    DESTINATION include/hppModel
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
