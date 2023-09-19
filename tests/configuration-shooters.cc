// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE configuration_shooters
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/configuration-shooter/gaussian.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/configuration-shooter/uniform-tpl.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

using hpp::pinocchio::DevicePtr_t;

using namespace hpp::core::configurationShooter;

namespace pin_test = hpp::pinocchio::unittest;


hpp::pinocchio::DevicePtr_t makeDevice() {
  using namespace hpp::pinocchio;

  DevicePtr_t device = Device::create("test");
  urdf::loadModelFromString(device, 0, "", "prismatic_x",
      "<robot name='n'><link name='base_link'/></robot>", "");

  return device;
}

template <typename CS_t>
void basic_test(CS_t cs, DevicePtr_t robot) {
  hpp::core::Configuration_t q;
  for (int i = 0; i < 10; ++i) {
    cs->shoot(q);
    hpp::pinocchio::ArrayXb unused(robot->numberDof());
    BOOST_CHECK(!hpp::pinocchio::saturate(robot, q, unused));
  }
}

BOOST_AUTO_TEST_CASE(uniform) {
  DevicePtr_t robot = pin_test::makeDevice(pin_test::HumanoidSimple);
  UniformPtr_t cs = Uniform::create(robot);

  basic_test(cs, robot);
}

BOOST_AUTO_TEST_CASE(gaussian) {
  DevicePtr_t robot = pin_test::makeDevice(pin_test::HumanoidSimple);

  GaussianPtr_t cs = Gaussian::create(robot);

  basic_test(cs, robot);

  cs->sigma(0);

  hpp::core::Configuration_t q;
  cs->shoot(q);
  BOOST_CHECK(q.isApprox(cs->center()));
}

BOOST_AUTO_TEST_CASE(uniform_seedable) {
  DevicePtr_t robot = makeDevice();

  robot->model().lowerPositionLimit[0] = -1.0;
  robot->model().upperPositionLimit[0] = 1.0;

  UniformSeedable::Ptr_t shooter = UniformSeedable::create(robot);
  basic_test(shooter, robot);

  hpp::core::Configuration_t q;

  hpp::core::vector_t shoots (100);

  shooter->seed(0);
  for (int i = 0; i < shoots.size(); ++i) {
    shooter->shoot(q);
    shoots[i] = q[0];
    BOOST_CHECK_LE(q[0], 1.0);
    BOOST_CHECK_GE(q[0], -1.0);
  }

  // Check that setting the seed gives the same sequence of random values.
  shooter->seed(0);
  for (int i = 0; i < shoots.size(); ++i) {
    shooter->shoot(q);
    BOOST_CHECK_EQUAL(shoots[i], q[0]);
  }
}
