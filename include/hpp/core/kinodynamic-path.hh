// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
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

#ifndef HPP_CORE_KINODYNAMIC_PATH_HH
#define HPP_CORE_KINODYNAMIC_PATH_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/pinocchio/configuration.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Kino-dynamic straight path
///
/// This Path has the same behavior as the \link hpp::core::StraightPath
/// StraightPath \endlink class except for the translation part of the
/// free-flyer. For the translation part of the free-flyer KinodynamicPath store
/// a "bang-bang" trajectory dependent on time with either 2 segment of constant
/// acceleration or 3 segments with a constant velocity segment
///
/// In current implementation, only the translation part of the freeflyer joint
/// is considered by this class. The value of all other joint are interpolated
/// between the initial and end value using the interpolate() method.
///
/// The current implementation assume that :
/// * The first joint of the robot is a freeflyer
/// * The robot have an extra Config Space of dimension >= 6.
/// The first 3 values of the extraConfig are the velocity of the root and the 3
/// other values are the aceleration.
class HPP_CORE_DLLAPI KinodynamicPath : public StraightPath {
 public:
  typedef StraightPath parent_t;
  /// Destructor
  virtual ~KinodynamicPath() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static KinodynamicPathPtr_t create(const DevicePtr_t& device,
                                     ConfigurationIn_t init,
                                     ConfigurationIn_t end, value_type length,
                                     ConfigurationIn_t a1, ConfigurationIn_t t0,
                                     ConfigurationIn_t t1, ConfigurationIn_t tv,
                                     ConfigurationIn_t t2,
                                     ConfigurationIn_t vLim) {
    KinodynamicPath* ptr = new KinodynamicPath(device, init, end, length, a1,
                                               t0, t1, tv, t2, vLim);
    KinodynamicPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  /// \param constraints the path is subject to
  static KinodynamicPathPtr_t create(
      const DevicePtr_t& device, ConfigurationIn_t init, ConfigurationIn_t end,
      value_type length, ConfigurationIn_t a1, ConfigurationIn_t t0,
      ConfigurationIn_t t1, ConfigurationIn_t tv, ConfigurationIn_t t2,
      ConfigurationIn_t vLim, ConstraintSetPtr_t constraints) {
    KinodynamicPath* ptr = new KinodynamicPath(
        device, init, end, length, a1, t0, t1, tv, t2, vLim, constraints);
    KinodynamicPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  static KinodynamicPathPtr_t createCopy(const KinodynamicPathPtr_t& path) {
    KinodynamicPath* ptr = new KinodynamicPath(*path);
    KinodynamicPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static KinodynamicPathPtr_t createCopy(
      const KinodynamicPathPtr_t& path, const ConstraintSetPtr_t& constraints) {
    KinodynamicPath* ptr = new KinodynamicPath(*path, constraints);
    KinodynamicPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Return a shared pointer to this
  ///
  /// As StaightPath are immutable, and refered to by shared pointers,
  /// they do not need to be copied.
  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Return a shared pointer to a copy of this and set constraints
  ///
  /// \param constraints constraints to apply to the copy
  /// \pre *this should not have constraints.
  virtual PathPtr_t copy(const ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }

  /// Extraction/Reversion of a sub-path
  /// \param subInterval interval of definition of the extract path
  /// If upper bound of subInterval is smaller than lower bound, return an empty
  /// path
  virtual PathPtr_t impl_extract(const interval_t& paramInterval) const;

  vector_t getT0() { return t0_; }

  vector_t getT1() { return t1_; }

  vector_t getT2() { return t2_; }

  vector_t getTv() { return tv_; }

  vector_t getA1() { return a1_; }

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "KinodynamicPath:" << std::endl;
    os << "interval: [ " << timeRange().first << ", " << timeRange().second
       << " ]" << std::endl;
    os << "initial configuration: " << pinocchio::displayConfig(initial_)
       << std::endl;
    os << "final configuration:   " << pinocchio::displayConfig(end_)
       << std::endl;
    return os;
  }
  /// Constructor
  KinodynamicPath(const DevicePtr_t& robot, ConfigurationIn_t init,
                  ConfigurationIn_t end, value_type length,
                  ConfigurationIn_t a1, ConfigurationIn_t t0,
                  ConfigurationIn_t t1, ConfigurationIn_t tv,
                  ConfigurationIn_t t2, ConfigurationIn_t vLim);

  /// Constructor with constraints
  KinodynamicPath(const DevicePtr_t& robot, ConfigurationIn_t init,
                  ConfigurationIn_t end, value_type length,
                  ConfigurationIn_t a1, ConfigurationIn_t t0,
                  ConfigurationIn_t t1, ConfigurationIn_t tv,
                  ConfigurationIn_t t2, ConfigurationIn_t vLim,
                  ConstraintSetPtr_t constraints);

  /// Copy constructor
  KinodynamicPath(const KinodynamicPath& path);

  /// Copy constructor with constraints
  KinodynamicPath(const KinodynamicPath& path,
                  const ConstraintSetPtr_t& constraints);

  void init(KinodynamicPathPtr_t self) {
    parent_t::init(self);
    weak_ = self;
    checkPath();
  }

  virtual bool impl_compute(ConfigurationOut_t result, value_type t) const;

  inline double sgnenum(double val) const { return ((0. < val) - (val < 0.)); }

  inline int sgn(double d) const { return d >= 0.0 ? 1 : -1; }

  inline double sgnf(double d) const { return d >= 0.0 ? 1.0 : -1.0; }

  const DevicePtr_t& device() const { return device_; }

 private:
  KinodynamicPathWkPtr_t weak_;
  DevicePtr_t device_;
  vector_t a1_;
  vector_t t0_, t1_, tv_, t2_, vLim_;
};  // class KinodynamicPath
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_KINODYNAMIC_PATH_HH
