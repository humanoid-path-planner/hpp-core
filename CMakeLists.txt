#
# Copyright (c) 2010, 2011, 2020 CNRS Authors: Florent Lamiraux, Joseph Mirabel,
# Guilhem Saurel
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

cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_USE_TEMPLATE_CSS TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

set(PROJECT_NAME hpp-core)
set(PROJECT_DESCRIPTION
    "Implement basic classes for canonical path planning for kinematic chains.")

include(cmake/hpp.cmake)
include(cmake/boost.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake-modules/ ${CMAKE_MODULE_PATH})

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_project_dependency(hpp-pinocchio REQUIRED)
add_project_dependency(hpp-util REQUIRED)
add_project_dependency(hpp-statistics REQUIRED)
add_project_dependency(hpp-constraints REQUIRED)
add_project_dependency(proxsuite REQUIRED)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# Declare Headers
set(${PROJECT_NAME}_HEADERS
    include/hpp/core/bi-rrt-planner.hh
    include/hpp/core/collision-pair.hh
    include/hpp/core/collision-path-validation-report.hh
    include/hpp/core/collision-validation.hh
    include/hpp/core/relative-motion.hh
    include/hpp/core/collision-validation-report.hh
    include/hpp/core/projection-error.hh
    include/hpp/core/configuration-shooter.hh
    include/hpp/core/configuration-shooter/uniform.hh
    include/hpp/core/configuration-shooter/uniform-tpl.hh
    include/hpp/core/configuration-shooter/uniform-tpl.hxx
    include/hpp/core/configuration-shooter/gaussian.hh
    include/hpp/core/config-projector.hh
    include/hpp/core/config-validation.hh
    include/hpp/core/config-validations.hh
    include/hpp/core/connected-component.hh
    include/hpp/core/constraint.hh
    include/hpp/core/constraint-set.hh
    include/hpp/core/container.hh
    include/hpp/core/continuous-validation.hh
    include/hpp/core/continuous-validation/dichotomy.hh
    include/hpp/core/continuous-validation/progressive.hh
    include/hpp/core/continuous-validation/interval-validation.hh
    include/hpp/core/continuous-validation/body-pair-collision.hh
    include/hpp/core/continuous-validation/solid-solid-collision.hh
    include/hpp/core/diffusing-planner.hh
    include/hpp/core/distance/reeds-shepp.hh
    include/hpp/core/distance.hh
    include/hpp/core/distance-between-objects.hh
    include/hpp/core/dubins-path.hh
    include/hpp/core/edge.hh
    include/hpp/core/fwd.hh
    include/hpp/core/joint-bound-validation.hh
    include/hpp/core/obstacle-user.hh
    include/hpp/core/path-validations.hh
    include/hpp/core/path-validation/discretized.hh
    include/hpp/core/path-validation/discretized-collision-checking.hh
    include/hpp/core/path-validation/discretized-joint-bound.hh
    include/hpp/core/node.hh
    include/hpp/core/parameter.hh
    include/hpp/core/path.hh
    include/hpp/core/path-optimization/linear-constraint.hh
    include/hpp/core/path-optimization/partial-shortcut.hh
    include/hpp/core/path-optimization/quadratic-program.hh
    include/hpp/core/path-optimization/random-shortcut.hh
    include/hpp/core/path-optimization/simple-shortcut.hh
    include/hpp/core/path-optimization/simple-time-parameterization.hh
    include/hpp/core/path-optimization/spline-gradient-based.hh
    include/hpp/core/path-optimization/spline-gradient-based-abstract.hh
    include/hpp/core/path-optimizer.hh
    include/hpp/core/path-planner.hh
    include/hpp/core/path-planning-failed.hh
    include/hpp/core/path-planner/k-prm-star.hh
    include/hpp/core/path-planner/bi-rrt-star.hh
    include/hpp/core/path-validation.hh
    include/hpp/core/path-validation-report.hh
    include/hpp/core/path-vector.hh
    include/hpp/core/path/spline.hh
    include/hpp/core/path/hermite.hh
    include/hpp/core/path/math.hh
    include/hpp/core/kinodynamic-path.hh
    include/hpp/core/kinodynamic-oriented-path.hh
    include/hpp/core/plan-and-optimize.hh
    include/hpp/core/plugin.hh
    include/hpp/core/problem.hh
    include/hpp/core/problem-solver.hh
    include/hpp/core/roadmap.hh
    include/hpp/core/steering-method.hh
    include/hpp/core/steering-method/fwd.hh
    include/hpp/core/steering-method/straight.hh
    include/hpp/core/steering-method/car-like.hh
    include/hpp/core/steering-method/constant-curvature.hh
    include/hpp/core/steering-method/dubins.hh
    include/hpp/core/steering-method/hermite.hh
    include/hpp/core/steering-method/reeds-shepp.hh
    include/hpp/core/steering-method/spline.hh
    include/hpp/core/steering-method/snibud.hh
    include/hpp/core/steering-method/reeds-shepp.hh
    include/hpp/core/steering-method/steering-kinodynamic.hh
    include/hpp/core/straight-path.hh
    include/hpp/core/interpolated-path.hh
    include/hpp/core/validation-report.hh
    include/hpp/core/visibility-prm-planner.hh
    include/hpp/core/weighed-distance.hh
    include/hpp/core/kinodynamic-distance.hh
    include/hpp/core/path-projector/progressive.hh
    include/hpp/core/path-projector/dichotomy.hh
    include/hpp/core/path-projector/global.hh
    include/hpp/core/path-projector/recursive-hermite.hh
    include/hpp/core/path-projector.hh
    include/hpp/core/nearest-neighbor.hh
    include/hpp/core/parser/roadmap.hh
    include/hpp/core/problem-target.hh
    include/hpp/core/problem-target/goal-configurations.hh
    include/hpp/core/problem-target/task-target.hh
    include/hpp/core/subchain-path.hh
    include/hpp/core/time-parameterization.hh
    include/hpp/core/time-parameterization/piecewise-polynomial.hh
    include/hpp/core/time-parameterization/polynomial.hh)

set(${PROJECT_NAME}_SOURCES
    src/astar.hh
    src/bi-rrt-planner.cc
    src/collision-validation.cc
    src/configuration-shooter/uniform.cc
    src/configuration-shooter/uniform-tpl.cc
    src/configuration-shooter/gaussian.cc
    src/config-projector.cc
    src/config-validations.cc
    src/connected-component.cc
    src/constraint.cc
    src/constraint-set.cc
    src/continuous-validation.cc
    src/continuous-validation/body-pair-collision.cc
    src/continuous-validation/dichotomy.cc
    src/continuous-validation/solid-solid-collision.cc
    src/continuous-validation/progressive.cc
    src/diffusing-planner.cc
    src/distance/serialization.cc
    src/distance/reeds-shepp.cc
    src/distance-between-objects.cc
    src/dubins.hh
    src/dubins.cc
    src/dubins-path.cc
    src/extracted-path.hh
    src/extracted-path.cc
    src/interpolated-path.cc
    src/joint-bound-validation.cc
    src/obstacle-user.cc
    src/path-validations.cc
    src/path-validation/discretized.cc
    src/path-validation/discretized-collision-checking.cc
    src/path-validation/discretized-joint-bound.cc
    src/path-validation/no-validation.hh
    src/nearest-neighbor/basic.hh #
    src/nearest-neighbor/basic.cc #
    # src/nearest-neighbor/k-d-tree.cc # src/nearest-neighbor/k-d-tree.hh #
    src/nearest-neighbor/serialization.cc #
    src/node.cc #
    src/parameter.cc #
    src/path.cc #
    src/path-optimizer.cc #
    src/path-optimization/linear-constraint.cc #
    src/path-optimization/spline-gradient-based-abstract.cc #
    src/path-optimization/partial-shortcut.cc #
    src/path-optimization/random-shortcut.cc
    src/path-optimization/simple-shortcut.cc
    src/path-optimization/simple-time-parameterization.cc #
    src/path-planner.cc #
    src/path-planner/k-prm-star.cc
    src/path-planner/bi-rrt-star.cc
    src/path-vector.cc #
    src/path/spline.cc
    src/path/hermite.cc
    src/plan-and-optimize.cc #
    src/plugin.cc # TODO : addObstacle
    src/problem.cc # TODO : addObstacle
    src/problem-solver.cc # TODO : addObstacle
    src/path-projector.cc
    src/path-projector/dichotomy.cc #
    src/path-projector/global.cc #
    src/path-projector/progressive.cc #
    src/path-projector/recursive-hermite.cc
    src/path-projector.cc #
    src/path-validation.cc
    src/problem-target/goal-configurations.cc #
    src/problem-target/task-target.cc #
    src/reeds-shepp-path.cc
    src/relative-motion.cc
    src/kinodynamic-path.cc
    src/kinodynamic-oriented-path.cc
    src/serialization.cc
    src/steering-method/steering-kinodynamic.cc
    src/roadmap.cc
    src/steering-method/reeds-shepp.cc # TODO access type of joint
    src/steering-method/car-like.cc
    src/steering-method/constant-curvature.cc
    src/steering-method/dubins.cc
    src/steering-method/snibud.cc
    src/steering-method/spline.cc
    src/steering-method/straight.cc
    src/straight-path.cc
    src/interpolated-path.cc
    src/visibility-prm-planner.cc
    src/weighed-distance.cc
    src/kinodynamic-distance.cc
    src/path-projector/progressive.cc
    src/path-projector/dichotomy.cc
    src/path-projector/global.cc
    src/path-projector.cc
    src/problem-target/goal-configurations.cc
    src/problem-target/task-target.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} ${CMAKE_DL_LIBS} hpp-util::hpp-util pinocchio::pinocchio
  hpp-statistics::hpp-statistics hpp-constraints::hpp-constraints)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# GPL licenced part
set(${PROJECT_NAME}_HEADERS_GPL
    src/path-optimization/spline-gradient-based/eiquadprog_2011.hpp)
set(${PROJECT_NAME}_HEADERS_GPL src/path-optimization/quadratic-program.cc
                                src/path-optimization/spline-gradient-based.cc)
add_library(${PROJECT_NAME}-gpl SHARED ${${PROJECT_NAME}_SOURCES_GPL}
                                       ${${PROJECT_NAME}_HEADERS_GPL})
target_include_directories(${PROJECT_NAME}-gpl PRIVATE src)
target_link_libraries(${PROJECT_NAME}-gpl ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}-gpl
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(plugins)
add_subdirectory(tests)

# Add dependency toward hpp-core library in pkg-config file.
pkg_config_append_libs("hpp-core")

if(INSTALL_DOCUMENTATION)
  install(
    FILES doc/continuous-validation.pdf
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html)
endif(INSTALL_DOCUMENTATION)

set(HPP_CORE_CMAKE_PLUGIN
    "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}-plugin.cmake"
)
set(PKG_CONFIG_EXTRA
    "${PKG_CONFIG_EXTRA}\ncmake_plugin=${HPP_CORE_CMAKE_PLUGIN}")
set(PACKAGE_EXTRA_MACROS "set(HPP_CORE_CMAKE_PLUGIN ${HPP_CORE_CMAKE_PLUGIN})")
install(
  FILES cmake-modules/hpp-core-plugin.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
