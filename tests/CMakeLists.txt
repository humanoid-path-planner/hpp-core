# Copyright 2012, 2013, 2014, 2020 CNRS-LAAS
#
# Author: Mathieu Geisert, Guilhem Saurel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

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
  TARGET_INCLUDE_DIRECTORIES(${NAME} PRIVATE ../src)
  TARGET_LINK_LIBRARIES(${NAME} Boost::unit_test_framework ${PROJECT_NAME}-gpl)
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
ADD_TESTCASE(reeds-and-shepp FALSE)
