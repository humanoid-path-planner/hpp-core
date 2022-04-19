//
// Copyright (c) 2018 CNRS
// Authors: Diane Bury
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#define BOOST_TEST_MODULE test_config_validations

#include <boost/test/included/unit_test.hpp>
#include <hpp/core/config-validations.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/problem.hh>
#include <pinocchio/fwd.hpp>

using namespace hpp::core;

typedef std::vector<std::string> ConfigValidationTypes_t;

BOOST_AUTO_TEST_CASE(config_validations) {
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->robot(ps->createRobot("robot"));
  ConfigValidationTypes_t configTypes = ps->configValidationTypes();
  BOOST_CHECK_MESSAGE(std::find(configTypes.begin(), configTypes.end(),
                                "CollisionValidation") != configTypes.end(),
                      "CollisionValidation is not present");
  BOOST_CHECK_MESSAGE(std::find(configTypes.begin(), configTypes.end(),
                                "JointBoundValidation") != configTypes.end(),
                      "JointBoundValidation is not present");

  ProblemPtr_t problem = ps->problem();
  ConfigValidationsPtr_t configValidations = problem->configValidations();
  BOOST_CHECK_MESSAGE(configValidations->numberConfigValidations() == 2,
                      "ConfigValidations not set right in Problem");
}

BOOST_AUTO_TEST_CASE(test_add_config_validation) {
  ProblemSolverPtr_t ps = ProblemSolver::create();
  ps->robot(ps->createRobot("robot"));
  ProblemPtr_t problem = ps->problem();

  ps->clearConfigValidations();
  BOOST_CHECK_MESSAGE(
      problem->configValidations()->numberConfigValidations() == 0,
      "Clearing ConfigValidations did not work");

  ps->addConfigValidation("CollisionValidation");
  BOOST_CHECK_MESSAGE(
      problem->configValidations()->numberConfigValidations() == 1,
      "Adding CollisionValidation to the ProblemSolver did not work");
}
