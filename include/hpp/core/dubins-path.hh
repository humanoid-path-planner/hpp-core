//
// Copyright (c) 2017 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_DUBINS_PATH_HH
#define HPP_CORE_DUBINS_PATH_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Car like motion going only forward
///
/// Implement a Dubins motion generation on the base joint.
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
/// The following interpolation is made:
///   \li Reeds and Shepp interpolation for the base_joint_xy and
///       base_joint_rz
///   \li If the wheel joints are passed using setWheelJoints,
///       the configuration parameter of those joints are computed so that
///       the wheel is aligned with the velocity.
///   \li linear interpolation for the other joints
class DubinsPath : public PathVector {
 public:
  typedef core::PathVector parent_t;

  /// Destructor
  virtual ~DubinsPath() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param extraLength Part of path length due to non Dubins degrees of
  ///        freedom,
  /// \param rho The radius of a turn,
  /// \param xyId, rzId indices in configuration vector of translation
  ///                   and rotation of the car,
  /// \param wheels vector of joints that represent turning wheels.
  static DubinsPathPtr_t create(const DevicePtr_t& device,
                                ConfigurationIn_t init, ConfigurationIn_t end,
                                value_type extraLength, value_type rho,
                                size_type xyId, size_type rzId,
                                const std::vector<JointPtr_t> wheels);
  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param extraLength Part of path length due to non Dubins degrees of
  ///        freedom,
  /// \param rho The radius of a turn.
  /// \param xyId, rzId indices in configuration vector of translation
  ///                   and rotation of the car,
  /// \param wheels vector of joints that represent turning wheels,
  /// \param constraints the path is subject to
  static DubinsPathPtr_t create(const DevicePtr_t& device,
                                ConfigurationIn_t init, ConfigurationIn_t end,
                                value_type extraLength, value_type rho,
                                size_type xyId, size_type rzId,
                                const std::vector<JointPtr_t> wheels,
                                ConstraintSetPtr_t constraints);

  /// Create copy and return shared pointer
  /// \param path path to copy
  static DubinsPathPtr_t createCopy(const DubinsPathPtr_t& path) {
    DubinsPath* ptr = new DubinsPath(*path);
    DubinsPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static DubinsPathPtr_t createCopy(const DubinsPathPtr_t& path,
                                    const ConstraintSetPtr_t& constraints) {
    DubinsPath* ptr = new DubinsPath(*path, constraints);
    DubinsPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Return a shared pointer to a copy of this object
  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Return a shared pointer to a copy of this and set constraints
  ///
  /// \param constraints constraints to apply to the copy
  /// \pre *this should not have constraints.
  virtual PathPtr_t copy(const ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }

  /// Return the internal robot.
  inline DevicePtr_t device() const { return device_; }

  /// Get the initial configuration
  inline Configuration_t initial() const { return initial_; }

  /// Get the final configuration
  inline Configuration_t end() const { return end_; }

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "DubinsPath:" << std::endl;
    Path::print(os);
    os << "initial configuration: " << initial_.transpose() << std::endl;
    os << "final configuration:   " << end_.transpose() << std::endl;
    return os;
  }
  /// Constructor
  DubinsPath(const DevicePtr_t& robot, ConfigurationIn_t init,
             ConfigurationIn_t end, value_type extraLength, value_type rho,
             size_type xyId, size_type rzId,
             const std::vector<JointPtr_t> wheels);

  /// Constructor with constraints
  DubinsPath(const DevicePtr_t& robot, ConfigurationIn_t init,
             ConfigurationIn_t end, value_type extraLength, value_type rho,
             size_type xyId, size_type rzId,
             const std::vector<JointPtr_t> wheels,
             ConstraintSetPtr_t constraints);

  /// Copy constructor
  DubinsPath(const DubinsPath& path);

  /// Copy constructor with constraints
  DubinsPath(const DubinsPath& path, const ConstraintSetPtr_t& constraints);

  void init(DubinsPathPtr_t self);

  /// For serialization only.
  DubinsPath() : xyId_(0), rzId_(0) {}

 private:
  void dubins_init_normalised(double alpha, double beta, double d);
  void dubins_init(vector3_t q0, vector3_t q1);
  typedef Eigen::Matrix<value_type, 3, 1> Lengths_t;

  DevicePtr_t device_;
  Configuration_t initial_;
  Configuration_t end_;
  const size_type xyId_, rzId_;
  size_type dxyId_, drzId_;
  std::vector<JointPtr_t> wheels_;
  std::size_t typeId_;
  Lengths_t lengths_;
  value_type extraLength_, rho_;

  vector3_t qi_;  // the initial configuration
  DubinsPathWkPtr_t weak_;

  HPP_SERIALIZABLE();
};  // class DubinsPath

/// \}
}  //   namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::core::DubinsPath)

#endif  // HPP_CORE_DUBINS_PATH_HH
