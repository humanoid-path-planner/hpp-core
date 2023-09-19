// Copyright (c) 2023 Eureka Robotics
// Authors: Joseph Mirabel

#include <hpp/core/configuration-shooter/uniform-tpl.hh>
#include <hpp/core/configuration-shooter/uniform-tpl.hxx>

namespace hpp {
namespace core {
namespace configurationShooter {

template class UniformTpl<std::default_random_engine>;

}  // namespace configurationShooter
}  // namespace core
}  // namespace hpp
