# Copyright (c) 2019, Joseph Mirabel Authors: Joseph Mirabel
# (joseph.mirabel@laas.fr)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

macro(HPP_ADD_PLUGIN PLUGIN_NAME)
  set(options EXCLUDE_FROM_ALL)
  set(oneValueArgs)
  set(multiValueArgs SOURCES LINK_DEPENDENCIES PKG_CONFIG_DEPENDENCIES EXPORT)
  cmake_parse_arguments(PLUGIN "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})
  if(PLUGIN_EXCLUDE_FROM_ALL)
    set(_options ${_options} EXCLUDE_FROM_ALL)
  endif()
  add_library(${PLUGIN_NAME} MODULE ${_options} ${PLUGIN_SOURCES})
  set_target_properties(${PLUGIN_NAME} PROPERTIES PREFIX ""
                                                  BUILD_WITH_INSTALL_RPATH TRUE)

  target_link_libraries(${PLUGIN_NAME} ${PLUGIN_LINK_DEPENDENCIES})
  foreach(DEP ${PLUGIN_PKG_CONFIG_DEPENDENCIES})
    pkg_config_use_dependency(${PLUGIN_NAME} ${DEP})
  endforeach()

  if(NOT PLUGIN_EXCLUDE_FROM_ALL)
    install(
      TARGETS ${PLUGIN_NAME}
      EXPORT ${PLUGIN_EXPORT}
      DESTINATION lib/hppPlugins)
  endif()
endmacro()

macro(ADD_PLUGIN PLUGIN_NAME)
  message(
    AUTHOR_WARNING
      "Macro ADD_PLUGIN is deprecated and should be replaced by HPP_ADD_PLUGIN")
  hpp_add_plugin(${PLUGIN_NAME} ${ARGN})
endmacro()
