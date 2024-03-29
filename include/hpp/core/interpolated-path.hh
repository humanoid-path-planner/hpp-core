// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_INTERPOLATED_PATH_HH
#define HPP_CORE_INTERPOLATED_PATH_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Piecewise linear interpolation between two configurations
///
/// This type of path is the return type of PathProjector algorithms.
///
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
///   \li linear interpolation for translation joints, bounded rotation
///       joints, and translation part of freeflyer joints,
///   \li angular interpolation for unbounded rotation joints,
///   \li constant angular velocity for SO(3) part of freeflyer joints.
class HPP_CORE_DLLAPI InterpolatedPath : public Path {
 public:
  typedef std::pair<const value_type, Configuration_t> InterpolationPoint_t;
  typedef std::map<value_type, Configuration_t, std::less<value_type>,
                   Eigen::aligned_allocator<InterpolationPoint_t> >
      InterpolationPoints_t;
  typedef Path parent_t;

  /// Destructor
  virtual ~InterpolatedPath() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param timeRange interval of definition
  static InterpolatedPathPtr_t create(const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      interval_t timeRange) {
    InterpolatedPath* ptr = new InterpolatedPath(device, init, end, timeRange);
    InterpolatedPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param timeRange interval of definition
  /// \param constraints the path is subject to
  static InterpolatedPathPtr_t create(const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      interval_t timeRange,
                                      ConstraintSetPtr_t constraints) {
    InterpolatedPath* ptr =
        new InterpolatedPath(device, init, end, timeRange, constraints);
    InterpolatedPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static InterpolatedPathPtr_t create(const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end,
                                      value_type length) {
    return create(device, init, end, interval_t(0, length));
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  /// \param constraints the path is subject to
  static InterpolatedPathPtr_t create(const DevicePtr_t& device,
                                      ConfigurationIn_t init,
                                      ConfigurationIn_t end, value_type length,
                                      ConstraintSetPtr_t constraints) {
    return create(device, init, end, interval_t(0, length), constraints);
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  static InterpolatedPathPtr_t createCopy(const InterpolatedPathPtr_t& path) {
    InterpolatedPath* ptr = new InterpolatedPath(*path);
    InterpolatedPathPtr_t shPtr(ptr);
    ptr->initCopy(shPtr);
    return shPtr;
  }

  /// Create an interpolation of this path
  /// \param path path to interpolate
  /// \param nbSamples number of samples between the initial and end
  ///        configuration
  /// \note it is assume that the constraints are constant along the path
  static InterpolatedPathPtr_t create(const PathPtr_t& path,
                                      const DevicePtr_t& device,
                                      const std::size_t& nbSamples);

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static InterpolatedPathPtr_t createCopy(
      const InterpolatedPathPtr_t& path,
      const ConstraintSetPtr_t& constraints) {
    InterpolatedPath* ptr = new InterpolatedPath(*path, constraints);
    InterpolatedPathPtr_t shPtr(ptr);
    ptr->initCopy(shPtr);
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

  virtual PathPtr_t reverse() const;

  /// Return the internal robot.
  DevicePtr_t device() const;

  /// Insert interpolation point
  void insert(const value_type& time, ConfigurationIn_t config) {
    configs_.insert(InterpolationPoint_t(time, config));
  }

  /// Get the initial configuration
  Configuration_t initial() const { return configs_.begin()->second; }

  /// Get the final configuration
  Configuration_t end() const { return configs_.rbegin()->second; }

  const InterpolationPoints_t& interpolationPoints() const { return configs_; }

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "InterpolatedPath:" << std::endl;
    Path::print(os);
    os << "initial configuration: " << initial().transpose() << std::endl;
    os << "final configuration:   " << end().transpose() << std::endl;
    return os;
  }

  /// Constructor
  InterpolatedPath(const DevicePtr_t& robot, ConfigurationIn_t init,
                   ConfigurationIn_t end, interval_t timeRange);

  /// Constructor with constraints
  InterpolatedPath(const DevicePtr_t& robot, ConfigurationIn_t init,
                   ConfigurationIn_t end, interval_t timeRange,
                   ConstraintSetPtr_t constraints);

  /// DIscretization of a given path.
  InterpolatedPath(const PathPtr_t& path, const DevicePtr_t& device,
                   const std::size_t& nbSamples);

  /// Copy constructor
  InterpolatedPath(const InterpolatedPath& path);

  /// Copy constructor with constraints
  InterpolatedPath(const InterpolatedPath& path,
                   const ConstraintSetPtr_t& constraints);

  void init(InterpolatedPathPtr_t self);

  void initCopy(InterpolatedPathPtr_t self);

  virtual bool impl_compute(ConfigurationOut_t result, value_type param) const;
  /// Virtual implementation of derivative
  virtual void impl_derivative(vectorOut_t result, const value_type& t,
                               size_type order) const;
  virtual void impl_velocityBound(vectorOut_t result, const value_type& t0,
                                  const value_type& t1) const;

  /// Extraction/Reversion of a sub-path
  /// See Path::extract
  PathPtr_t impl_extract(const interval_t& subInterval) const;

 private:
  DevicePtr_t device_;
  InterpolationPoints_t configs_;
  InterpolatedPathWkPtr_t weak_;
};  // class InterpolatedPath
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_INTERPOLATED_PATH_HH
