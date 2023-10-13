//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_DOF_EXTRACTED_PATH_HH
#define HPP_CORE_DOF_EXTRACTED_PATH_HH

#include <hpp/constraints/matrix-view.hh>
#include <hpp/core/path.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Result of the selection of some configuration parameter of an original path
/// \note Decorator design pattern
/// \todo the configuration parameter cannot be rearranged.
class SubchainPath : public Path {
 public:
  typedef Path parent_t;

  virtual ~SubchainPath() {}

  /// Return a shared pointer to a copy of this
  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Return a shared pointer to a copy of this and set constraints
  ///
  /// \param constraints constraints to apply to the copy
  /// \pre *this should not have constraints.
  virtual PathPtr_t copy(const ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }

  /// \copydoc SubchainPath::SubchainPath
  static SubchainPathPtr_t create(const PathPtr_t& original,
                                  const segments_t& confIntervals,
                                  const segments_t& velIntervals) {
    SubchainPath* ptr = new SubchainPath(original, confIntervals, velIntervals);
    SubchainPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  static SubchainPathPtr_t createCopy(const SubchainPathPtr_t& path) {
    SubchainPath* ptr = new SubchainPath(*path);
    SubchainPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  static SubchainPathPtr_t createCopy(const SubchainPathPtr_t& path,
                                      const ConstraintSetPtr_t& constraints) {
    SubchainPath* ptr = new SubchainPath(*path, constraints);
    SubchainPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  virtual bool impl_compute(ConfigurationOut_t result, value_type param) const {
    bool success = original_->eval(q_, param);
    if (success) dofExtract(q_, result);
    return success;
  }

  /// Get the initial configuration
  inline Configuration_t initial() const {
    Configuration_t q(outputSize());
    dofExtract(original_->initial(), q);
    return q;
  }

  /// Get the final configuration
  inline Configuration_t end() const {
    Configuration_t q(outputSize());
    dofExtract(original_->end(), q);
    return q;
  }

  void dofExtract(ConfigurationIn_t qin, ConfigurationOut_t qout) const {
    qout = configView_.rview(qin);
  }

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "Dof Extracted Path:" << std::endl;
    os << "intervals: " << configView_ << std::endl;
    os << "original path:" << std::endl;
    os << *original_ << std::endl;
    return os;
  }

  /// Constructor
  ///
  /// \param original Path to extract,
  /// \param confIntervals of the configuration parameters to be extracted
  /// \param velIntervals of the configuration parameters to be extracted
  SubchainPath(const PathPtr_t& original, const segments_t& confIntervals,
               const segments_t& velIntervals)
      : Path(original->timeRange(), Eigen::BlockIndex::cardinal(confIntervals),
             Eigen::BlockIndex::cardinal(velIntervals)),
        original_(original),
        configView_(confIntervals),
        q_(Configuration_t::Zero(original->outputSize())) {}

  SubchainPath(const SubchainPath& path)
      : Path(path),
        original_(path.original_),
        configView_(path.configView_),
        q_(path.q_),
        weak_() {}

  SubchainPath(const SubchainPath& path, const ConstraintSetPtr_t& constraints)
      : Path(path, constraints),
        original_(path.original_),
        configView_(path.configView_),
        weak_() {}

  void init(SubchainPathPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }

 private:
  PathPtr_t original_;
  Eigen::RowBlockIndices configView_, velView_;
  mutable Configuration_t q_;
  SubchainPathWkPtr_t weak_;
};  // SubchainPath
/// \}
}  //   namespace core
}  // namespace hpp
#endif  // HPP_CORE_DOF_EXTRACTED_PATH_HH
