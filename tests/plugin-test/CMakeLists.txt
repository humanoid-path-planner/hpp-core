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

ADD_LIBRARY(example MODULE EXCLUDE_FROM_ALL example.cc)
SET_TARGET_PROPERTIES(example PROPERTIES PREFIX ""
  #BUILD_WITH_INSTALL_RPATH TRUE
  )

TARGET_LINK_LIBRARIES(example ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY(example hpp-pinocchio)
#INSTALL(TARGETS example DESTINATION lib/hppPlugins)
