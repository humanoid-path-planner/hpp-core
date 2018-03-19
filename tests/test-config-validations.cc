//
// Copyright (c) 2018 CNRS
// Authors: Diane Bury
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE test_config_validations

#include <boost/test/included/unit_test.hpp>

#include <hpp/core/problem.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/config-validations.hh>

using namespace hpp::core;

typedef std::vector <std::string> ConfigValidationTypes_t;

BOOST_AUTO_TEST_CASE (config_validations)
{
    ProblemSolverPtr_t ps = ProblemSolver::create ();
    ps->robot ( ps->createRobot ("robot") );
    ConfigValidationTypes_t configTypes = ps->configValidationTypes ();
    BOOST_CHECK_MESSAGE (std::find(configTypes.begin(), configTypes.end(),"CollisionValidation")
        !=configTypes.end(), "CollisionValidation is not present");
    BOOST_CHECK_MESSAGE (std::find(configTypes.begin(), configTypes.end(),"JointBoundValidation")
        !=configTypes.end(), "JointBoundValidation is not present");

    ProblemPtr_t problem = ps->problem ();
    ConfigValidationsPtr_t configValidations = problem->configValidations ();
    BOOST_CHECK_MESSAGE (configValidations->numberConfigValidations () == 2,
        "ConfigValidations not set right in Problem");
}

BOOST_AUTO_TEST_CASE ( test_add_config_validation )
{
    ProblemSolverPtr_t ps = ProblemSolver::create ();
    ps->robot ( ps->createRobot ("robot") );
    ProblemPtr_t problem = ps->problem ();
    ConfigValidationsPtr_t configValidations = problem->configValidations ();

    ps->clearConfigValidations ();
    BOOST_CHECK_MESSAGE (configValidations->numberConfigValidations () == 0,
        "Clearing ConfigValidations did not work");

    ps->addConfigValidation ("CollisionValidation");
    BOOST_CHECK_MESSAGE (configValidations->numberConfigValidations () == 1,
        "Adding CollisionValidation to the ProblemSolver did not work");
}