//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_CORE_STRAIGHT_PATH_HH
#define HPP_CORE_STRAIGHT_PATH_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>
#include <hpp/pinocchio/liegroup-element.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Linear interpolation between two configurations
///
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
///   \li linear interpolation for translation joints, bounded rotation
///       joints, and translation part of freeflyer joints,
///   \li angular interpolation for unbounded rotation joints,
///   \li constant angular velocity for SO(3) part of freeflyer joints.
class HPP_CORE_DLLAPI StraightPath : public Path {
 public:
  typedef Path parent_t;
  /// Destructor
  virtual ~StraightPath() {}

  /// Create instance and return shared pointer
  /// \param init, end Start and end configurations of the path,
  /// \param interval interval of definition
  /// \param constraints the path is subject to.
  static StraightPathPtr_t create(
      LiegroupSpacePtr_t space, vectorIn_t init, vectorIn_t end,
      interval_t interval,
      ConstraintSetPtr_t constraints = ConstraintSetPtr_t()) {
    StraightPath* ptr;
    if (constraints)
      ptr = new StraightPath(space, init, end, interval, constraints);
    else
      ptr = new StraightPath(space, init, end, interval);
    StraightPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create instance and return shared pointer
  /// \param init, end Start and end configurations of the path
  /// \param interval interval of definition
  /// \param constraints the path is subject to.
  static StraightPathPtr_t create(
      LiegroupElementConstRef init, LiegroupElementConstRef end,
      interval_t interval,
      ConstraintSetPtr_t constraints = ConstraintSetPtr_t()) {
    assert(init.space() == end.space());
    return create(init.space(), init.vector(), end.vector(), interval,
                  constraints);
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static StraightPathPtr_t create(
      const DevicePtr_t& device, ConfigurationIn_t init, ConfigurationIn_t end,
      value_type length,
      ConstraintSetPtr_t constraints = ConstraintSetPtr_t()) {
    return create(device, init, end, interval_t(0, length), constraints);
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param interval interval of definition.
  /// \param constraints the path is subject to
  static StraightPathPtr_t create(
      const DevicePtr_t& device, ConfigurationIn_t init, ConfigurationIn_t end,
      interval_t interval,
      ConstraintSetPtr_t constraints = ConstraintSetPtr_t());

  /// Create copy and return shared pointer
  /// \param path path to copy
  static StraightPathPtr_t createCopy(const StraightPathPtr_t& path) {
    StraightPath* ptr = new StraightPath(*path);
    StraightPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static StraightPathPtr_t createCopy(const StraightPathPtr_t& path,
                                      const ConstraintSetPtr_t& constraints) {
    StraightPath* ptr = new StraightPath(*path, constraints);
    StraightPathPtr_t shPtr(ptr);
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

  /// Modify initial configuration
  /// \param initial new initial configuration
  /// \pre input configuration should be of the same size as current initial
  /// configuration
  void initialConfig(ConfigurationIn_t initial) {
    assert(initial.size() == initial_.size());
    initial_ = initial;
  }

  /// Modify end configuration
  /// \param end new end configuration
  /// \pre input configuration should be of the same size as current end
  /// configuration
  void endConfig(ConfigurationIn_t end) {
    assert(end.size() == end_.size());
    end_ = end;
  }

  /// Get the initial configuration
  Configuration_t initial() const { return initial_; }

  /// Get the final configuration
  Configuration_t end() const { return end_; }

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const;

  /// Constructor
  StraightPath(LiegroupSpacePtr_t space, vectorIn_t init, vectorIn_t end,
               interval_t interval);

  /// Constructor
  StraightPath(LiegroupSpacePtr_t space, vectorIn_t init, vectorIn_t end,
               interval_t interval, ConstraintSetPtr_t constraints);

  /// Copy constructor
  StraightPath(const StraightPath& path);

  /// Copy constructor with constraints
  StraightPath(const StraightPath& path, const ConstraintSetPtr_t& constraints);

  void init(StraightPathPtr_t self) {
    parent_t::init(self);
    weak_ = self;
    checkPath();
  }

  virtual bool impl_compute(ConfigurationOut_t result, value_type param) const;
  /// Virtual implementation of derivative
  virtual void impl_derivative(vectorOut_t result, const value_type& t,
                               size_type order) const;

  virtual void impl_velocityBound(vectorOut_t result, const value_type&,
                                  const value_type&) const;

  /// Extraction/Reversion of a sub-path
  /// \param subInterval interval of definition of the extract path
  /// If upper bound of subInterval is smaller than lower bound,
  /// result is reversed.
  PathPtr_t impl_extract(const interval_t& subInterval) const;

 protected:
  LiegroupSpacePtr_t space_;
  Configuration_t initial_;
  Configuration_t end_;

 private:
  StraightPathWkPtr_t weak_;

 protected:
  StraightPath() {}

 private:
  HPP_SERIALIZABLE();
};  // class StraightPath
/// \}
}  //   namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::core::StraightPath)

#endif  // HPP_CORE_STRAIGHT_PATH_HH
