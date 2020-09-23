# Copyright (c) 2019, 2020 Joseph Mirabel, Guilhem Saurel, CNRS
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

include(hpp-core-plugin)

hpp_add_plugin(spline-gradient-based
  EXPORT spline-gradient-basedTargets
  SOURCES spline-gradient-based.cc
  LINK_DEPENDENCIES PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}-gpl)

project_install_component(spline-gradient-based)

add_project_dependency(toppra FOR_COMPONENT toppra)
if(toppra_FOUND)
  message(STATUS "Found toppra ${toppra_VERSION}")
  hpp_add_plugin(toppra
    EXPORT toppraTargets
    SOURCES toppra
    LINK_DEPENDENCIES PUBLIC ${PROJECT_NAME} toppra::toppra)

  project_install_component(toppra)
  install(FILES toppra.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/core/path-optimization/)
endif()
