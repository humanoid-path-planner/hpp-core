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

#ifndef HPP_CORE_PATH_VECTOR_HH
#define HPP_CORE_PATH_VECTOR_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
namespace core {
/// \addtogroup path
/// \{

/// Concatenation of several paths
class HPP_CORE_DLLAPI PathVector : public Path {
 public:
  typedef Path parent_t;
  /// \name Construction, destruction, copy
  /// \{

  /// Create instance and return shared pointer
  static PathVectorPtr_t create(size_type outputSize,
                                size_type outputDerivativeSize) {
    PathVector* ptr = new PathVector(outputSize, outputDerivativeSize);
    PathVectorPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  static PathVectorPtr_t create(size_type outputSize,
                                size_type outputDerivativeSize,
                                const ConstraintSetPtr_t& constraint) {
    PathVector* ptr =
        new PathVector(outputSize, outputDerivativeSize, constraint);
    PathVectorPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  static PathVectorPtr_t createCopy(const PathVectorPtr_t& original) {
    PathVector* ptr = new PathVector(*original);
    PathVectorPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Create instance and return shared pointer
  static PathVectorPtr_t createCopy(const PathVectorPtr_t& original,
                                    const ConstraintSetPtr_t& constraints) {
    PathVector* ptr = new PathVector(*original, constraints);
    PathVectorPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Return a shared pointer to a copy of this
  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Return a shared pointer to a copy of this with constraints
  /// \param constraints constraints to apply to the copy
  /// \pre *this should not have constraints.
  virtual PathPtr_t copy(const ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }

  /// Destructor
  virtual ~PathVector() {}
  /// \}

  /// Get the number of sub paths
  std::size_t numberPaths() const { return paths_.size(); }

  /// Get a path in the vector
  ///
  /// \param rank rank of the path in the vector. Should be between 0 and
  ///        numberPaths ().
  /// \return shared pointer to a copy of the path at requested rank with
  ///         constraints applicable to the PathVector.
  PathPtr_t pathAtRank(std::size_t rank) const;

  /// Get rank of direct path in vector at param
  ///
  /// \param param parameter in interval of definition,
  /// \retval localParam parameter on sub-path
  /// \return rank of direct path in vector
  std::size_t rankAtParam(const value_type& param,
                          value_type& localParam) const;

  /// Append a path at the end of the vector
  void appendPath(const PathPtr_t& path);

  /// Concatenate two vectors of path
  /// \deprecated use void method concatenate (const PathVectorPtr_t& path)
  ///             instead.
  HPP_CORE_DEPRECATED void concatenate(const PathVector& path);

  /// Concatenate two vectors of path
  /// \param path path to append at the end of this one
  ///
  /// Each element of path is appended to this one.
  void concatenate(const PathVectorPtr_t& path);

  /// Get the initial configuration
  virtual Configuration_t initial() const { return paths_.front()->initial(); }

  /// Get the final configuration
  virtual Configuration_t end() const { return paths_.back()->end(); }

  /// Return the a path vector representing the same path but ensuring
  /// that there is no PathVector in the PathVector.
  void flatten(PathVectorPtr_t flattenedPath) const;

  /// Reversion of a path
  virtual PathPtr_t reverse() const;

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const;
  /// Constructor
  PathVector(std::size_t outputSize, std::size_t outputDerivativeSize)
      : parent_t(std::make_pair(0, 0), outputSize, outputDerivativeSize),
        paths_() {}
  /// Constructor
  PathVector(std::size_t outputSize, std::size_t outputDerivativeSize,
             const ConstraintSetPtr_t& constraint)
      : parent_t(std::make_pair(0, 0), outputSize, outputDerivativeSize,
                 constraint),
        paths_() {}
  /// Copy constructor
  PathVector(const PathVector& path) : parent_t(path), paths_() {
    assert(timeRange() == path.timeRange());
    for (Paths_t::const_iterator it = path.paths_.begin();
         it != path.paths_.end(); it++) {
      paths_.push_back((*it)->copy());
    }
  }

  /// Copy constructor with constraints
  PathVector(const PathVector& path, const ConstraintSetPtr_t& constraints)
      : parent_t(path, constraints), paths_() {
    assert(timeRange() == path.timeRange());
    for (Paths_t::const_iterator it = path.paths_.begin();
         it != path.paths_.end(); it++) {
      paths_.push_back((*it)->copy());
    }
  }

  void init(PathVectorPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }
  virtual bool impl_compute(ConfigurationOut_t result, value_type t) const;
  /// Virtual implementation of derivative
  virtual void impl_derivative(vectorOut_t result, const value_type& t,
                               size_type order) const;
  /// Extraction of a sub-path
  /// \param subInterval interval of definition of the extract path
  virtual PathPtr_t impl_extract(const interval_t& subInterval) const;

  /// Virtual implementation of velocity bound
  virtual void impl_velocityBound(vectorOut_t bound, const value_type& param0,
                                  const value_type& param1) const;

 private:
  Paths_t paths_;
  PathVectorWkPtr_t weak_;

 protected:
  PathVector() {}

 private:
  HPP_SERIALIZABLE();
};  // class PathVector
/// \}
}  //   namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::core::PathVector)

#endif  // HPP_CORE_PATH_VECTOR_HH
