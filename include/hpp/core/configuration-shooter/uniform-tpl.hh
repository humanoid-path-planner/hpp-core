// Copyright (c) 2023 Eureka Robotics
// Authors: Joseph Mirabel

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HH
#define HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HH

#include <hpp/core/configuration-shooter.hh>
#include <hpp/pinocchio/device.hh>
#include <random>

namespace hpp {
namespace core {
namespace configurationShooter {

/// \addtogroup configuration_sampling
/// \{

/// Uniformly sample with bounds of degrees of freedom using a custom generator.
template<class generator_t>
class HPP_CORE_DLLAPI UniformTpl : public ConfigurationShooter {
 public:
  typedef shared_ptr<UniformTpl<generator_t>> Ptr_t;
  typedef weak_ptr<UniformTpl<generator_t>> WkPtr_t;

  static Ptr_t create(const DevicePtr_t& robot) {
    UniformTpl* ptr = new UniformTpl(robot);
    Ptr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Set the generator seed.
  void seed(typename generator_t::result_type seed) {
    generator_.seed(seed);
  }

 protected:
  /// Uniformly sample configuration space
  ///
  /// Note that translation joints have to be bounded.
  UniformTpl(const DevicePtr_t& robot) : robot_(robot) {}
  void init(const Ptr_t& self) {
    ConfigurationShooter::init(self);
    weak_ = self;
  }

  virtual void impl_shoot(Configuration_t& q) const;

 private:
  DevicePtr_t robot_;
  WkPtr_t weak_;

  // The generator must be mutable because impl_shoot is const and
  // generator_t::operator() is not.
  mutable generator_t generator_;
};  // class UniformTpl
/// \}

typedef UniformTpl<std::default_random_engine> UniformSeedable;

}  // namespace configurationShooter
}  //   namespace core
}  // namespace hpp

#endif  // HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_TPL_HH
