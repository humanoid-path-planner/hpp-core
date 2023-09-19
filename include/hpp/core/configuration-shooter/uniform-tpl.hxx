// Copyright (c) 2023 Eureka Robotics
// Authors: Joseph Mirabel

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HXX
#define HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HXX

#include <hpp/core/configuration-shooter/uniform-tpl.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace core {
namespace configurationShooter {

template <class generator_t>
void UniformTpl<generator_t>::impl_shoot(Configuration_t& config) const {
  typedef std::uniform_real_distribution<value_type> distribution_t;

  if (!robot_->configSpace()->isVectorSpace()) {
    throw std::invalid_argument(
        "The device config space must be a vector space"
        " to use UniformTpl");
  }

  config.resize(robot_->configSize());
  auto const& model = robot_->model();

  for (size_type i = 0; i < model.nq; ++i) {
    distribution_t distrib(model.lowerPositionLimit[i],
                           model.upperPositionLimit[i]);
    config[i] = distrib(generator_);
  }
  auto const& ecs = robot_->extraConfigSpace();
  for (size_type i = 0; i < ecs.dimension(); ++i) {
    distribution_t distrib(ecs.lower(i), ecs.upper(i));
    config[model.nq + i] = distrib(generator_);
  }
}

}  // namespace configurationShooter
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HXX
