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

INCLUDE(hpp-core-plugin)

HPP_ADD_PLUGIN(spline-gradient-based
  SOURCES spline-gradient-based.cc
  LINK_DEPENDENCIES ${PROJECT_NAME} ${PROJECT_NAME}-gpl
  PKG_CONFIG_DEPENDENCIES hpp-util hpp-pinocchio hpp-statistics hpp-constraints)
