# Copyright 2012, 2013, 2014 CNRS-LAAS
#
# Author: Mathieu Geisert
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-core  If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Specify the tests data files directory.
ADD_DEFINITIONS(-DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data")



# Make Boost.Test generates the main function in test cases.
#ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    ADD_UNIT_TEST(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  ENDIF()

  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-util)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-pinocchio)
  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES} hpp-core hpp-core-gpl
    )

ENDMACRO(ADD_TESTCASE)

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

# ADD_TESTCASE (test-kdTree FALSE)
ADD_TESTCASE (roadmap-1 FALSE)
ADD_TESTCASE (test-intervals FALSE)
ADD_TESTCASE (test-solid-solid-collision FALSE)
ADD_TESTCASE (test-gradient-based FALSE)
ADD_TESTCASE (test-configprojector FALSE)
ADD_TESTCASE (path-projectors FALSE) # link error
ADD_TESTCASE (containers FALSE)
ADD_TESTCASE (paths FALSE)
ADD_TESTCASE (relative-motion FALSE)
ADD_TESTCASE (test-path-extraction FALSE)
ADD_TESTCASE (test-continuous-validation FALSE)
ADD_TESTCASE (test-config-validations FALSE)

ADD_TESTCASE (explicit-relative-transformation FALSE)
SET_PROPERTY(TARGET explicit-relative-transformation APPEND_STRING PROPERTY COMPILE_FLAGS " -DDATA_DIR=\\\"${CMAKE_CURRENT_SOURCE_DIR}/data\\\"")

ADD_TESTCASE (spline-path FALSE)
ADD_TESTCASE (problem FALSE)
ADD_TESTCASE (time-parameterization FALSE)
ADD_TESTCASE (configuration-shooters FALSE)
# ADD_TESTCASE (hermite-path FALSE) Refactoring hpp-model -> hpp-pinocchio not done
ADD_TESTCASE (test-kinodynamic FALSE)
ADD_SUBDIRECTORY(plugin-test)
CONFIG_FILES (plugin.cc)
ADD_TESTCASE (plugin TRUE)
ADD_DEPENDENCIES (plugin example)
