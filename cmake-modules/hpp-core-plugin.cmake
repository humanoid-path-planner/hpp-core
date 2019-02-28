# Copyright (c) 2019, Joseph Mirabel
# Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
#
# This file is part of hpp-core.
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
# hpp-core. If not, see <http://www.gnu.org/licenses/>.

MACRO(HPP_ADD_PLUGIN PLUGIN_NAME)
  SET(options EXCLUDE_FROM_ALL)
  SET(oneValueArgs )
  SET(multiValueArgs
    SOURCES
    LINK_DEPENDENCIES
    PKG_CONFIG_DEPENDENCIES)
  CMAKE_PARSE_ARGUMENTS(PLUGIN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  IF(PLUGIN_EXCLUDE_FROM_ALL)
    SET(_options ${_options} EXCLUDE_FROM_ALL)
  ENDIF()
  ADD_LIBRARY(${PLUGIN_NAME} MODULE ${_options} ${PLUGIN_SOURCES})
  SET_TARGET_PROPERTIES(${PLUGIN_NAME} PROPERTIES PREFIX "" BUILD_WITH_INSTALL_RPATH TRUE)

  TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${PLUGIN_LINK_DEPENDENCIES})
  FOREACH(DEP ${PLUGIN_PKG_CONFIG_DEPENDENCIES})
    PKG_CONFIG_USE_DEPENDENCY(${PLUGIN_NAME} ${DEP})
  ENDFOREACH()

  IF(NOT PLUGIN_EXCLUDE_FROM_ALL)
    INSTALL(TARGETS ${PLUGIN_NAME} DESTINATION lib/hppPlugins)
  ENDIF()
ENDMACRO()

MACRO(ADD_PLUGIN PLUGIN_NAME)
  MESSAGE(AUTHOR_WARNING "Macro ADD_PLUGIN is deprecated and should be replaced by HPP_ADD_PLUGIN")
  HPP_ADD_PLUGIN(${PLUGIN_NAME} ${ARGN})
ENDMACRO()
