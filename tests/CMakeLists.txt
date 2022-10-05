# Copyright 2012, 2013, 2014, 2020 CNRS-LAAS
#
# Author: Mathieu Geisert, Guilhem Saurel
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

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Specify the tests data files directory.
add_definitions(-DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data")

# Make Boost.Test generates the main function in test cases.
# ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against Boost and add
# it to the test suite.
#
macro(ADD_TESTCASE NAME GENERATED)
  if(${GENERATED} STREQUAL TRUE)
    add_unit_test(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  else()
    add_unit_test(${NAME} ${NAME}.cc)
  endif()
  target_include_directories(${NAME} PRIVATE ../src Boost::unit_test_framework)
  target_link_libraries(${NAME} ${PROJECT_NAME}-gpl)
endmacro(ADD_TESTCASE)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ADD_TESTCASE (test-kdTree FALSE)
add_testcase(roadmap-1 FALSE)
add_testcase(test-intervals FALSE)
add_testcase(test-solid-solid-collision FALSE)
add_testcase(test-gradient-based FALSE)
add_testcase(test-configprojector FALSE)
add_testcase(path-projectors FALSE) # link error
add_testcase(containers FALSE)
add_testcase(paths FALSE)
add_testcase(relative-motion FALSE)
add_testcase(test-path-extraction FALSE)
add_testcase(test-continuous-validation FALSE)
add_testcase(test-config-validations FALSE)

add_testcase(explicit-relative-transformation FALSE)
set_property(
  TARGET explicit-relative-transformation
  APPEND_STRING
  PROPERTY COMPILE_FLAGS " -DDATA_DIR=\\\"${CMAKE_CURRENT_SOURCE_DIR}/data\\\"")

add_testcase(spline-path FALSE)
add_testcase(problem FALSE)
add_testcase(time-parameterization FALSE)
add_testcase(configuration-shooters FALSE)
# ADD_TESTCASE (hermite-path FALSE) Refactoring hpp-model -> hpp-pinocchio not
# done
add_testcase(test-kinodynamic FALSE)
add_subdirectory(plugin-test)
config_files(plugin.cc)
add_testcase(plugin TRUE)
add_dependencies(plugin example)
add_testcase(reeds-and-shepp FALSE)
