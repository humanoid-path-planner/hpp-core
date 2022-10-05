# Copyright (c) 2019, 2020 Joseph Mirabel, Guilhem Saurel, CNRS Authors: Joseph
# Mirabel (joseph.mirabel@laas.fr)
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

include(hpp-core-plugin)

hpp_add_plugin(
  spline-gradient-based
  EXPORT
  spline-gradient-basedTargets
  SOURCES
  spline-gradient-based.cc
  LINK_DEPENDENCIES
  PUBLIC
  ${PROJECT_NAME}
  ${PROJECT_NAME}-gpl)

project_install_component(spline-gradient-based)

add_project_dependency(toppra FOR_COMPONENT toppra)
if(toppra_FOUND)
  message(STATUS "Found toppra ${toppra_VERSION}")
  hpp_add_plugin(
    toppra
    EXPORT
    toppraTargets
    SOURCES
    toppra
    LINK_DEPENDENCIES
    PUBLIC
    ${PROJECT_NAME}
    toppra::toppra)

  project_install_component(toppra)
  install(FILES toppra.hh
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/core/path-optimization/)
endif()
