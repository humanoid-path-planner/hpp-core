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

#include <boost/serialization/vector.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/serialization.hh>

namespace hpp {
namespace core {
bool ConstraintSet::impl_compute(ConfigurationOut_t configuration) {
  for (Constraints_t::iterator itConstraint = constraints_.begin();
       itConstraint != constraints_.end(); ++itConstraint) {
    if (!(*itConstraint)->impl_compute(configuration)) return false;
  }
  return true;
}

ConstraintPtr_t ConstraintSet::copy() const { return createCopy(weak_.lock()); }

ConstraintSet::ConstraintSet(const DevicePtr_t& robot, const std::string& name)
    : Constraint(name), constraints_(), configProjI_(-1) {
  constraints_.push_back(ConfigProjector::create(robot, "Trivial", 1e-12, 0));
}

ConstraintSet::ConstraintSet(const ConstraintSet& other)
    : Constraint(other), constraints_(), configProjI_(other.configProjI_) {
  for (Constraints_t::const_iterator it = other.constraints_.begin();
       it != other.constraints_.end(); ++it)
    constraints_.push_back((*it)->copy());
}

void ConstraintSet::addConstraint(const ConstraintPtr_t& c) {
  ConfigProjectorPtr_t cp(HPP_DYNAMIC_PTR_CAST(ConfigProjector, c));
  ConstraintSetPtr_t cs(HPP_DYNAMIC_PTR_CAST(ConstraintSet, c));
  if (cp) {
    if (configProjI_ >= 0)
      throw std::runtime_error("Constraint set " + name() +
                               " cannot store "
                               "more than one config-projector.");
    constraints_.erase(constraints_.begin());
    configProjI_ = static_cast<int>(constraints_.size());
  } else if (cs) {
    for (Constraints_t::iterator _c = cs->begin(); _c != cs->end(); ++_c)
      addConstraint(*_c);
    return;
  }
  constraints_.push_back(c);
}

ConfigProjectorPtr_t ConstraintSet::configProjector() const {
  return (configProjI_ >= 0
              ? HPP_STATIC_PTR_CAST(ConfigProjector, constraints_[configProjI_])
              : ConfigProjectorPtr_t());
}

bool ConstraintSet::isSatisfied(ConfigurationIn_t configuration) {
  for (Constraints_t::iterator itConstraint = constraints_.begin();
       itConstraint != constraints_.end(); ++itConstraint) {
    if (!(*itConstraint)->isSatisfied(configuration)) {
      return false;
    }
  }
  return true;
}

bool ConstraintSet::isSatisfied(ConfigurationIn_t configuration,
                                vector_t& error) {
  bool result = true;
  error.resize(0);
  vector_t localError;
  for (Constraints_t::iterator itConstraint = constraints_.begin();
       itConstraint != constraints_.end(); ++itConstraint) {
    if (!(*itConstraint)->isSatisfied(configuration, localError)) {
      result = false;
    }
    error.conservativeResize(error.size() + localError.size());
    error.tail(localError.size()) = localError;
  }
  return result;
}

size_type ConstraintSet::numberNonLockedDof() const {
  return _configProj()->numberFreeVariables();
}

void ConstraintSet::compressVector(vectorIn_t normal, vectorOut_t small) const {
  _configProj()->compressVector(normal, small);
}

void ConstraintSet::uncompressVector(vectorIn_t small,
                                     vectorOut_t normal) const {
  _configProj()->uncompressVector(small, normal);
}

void ConstraintSet::compressMatrix(matrixIn_t normal, matrixOut_t small,
                                   bool rows) const {
  _configProj()->compressMatrix(normal, small, rows);
}

void ConstraintSet::uncompressMatrix(matrixIn_t small, matrixOut_t normal,
                                     bool rows) const {
  _configProj()->uncompressMatrix(small, normal, rows);
}

std::ostream& ConstraintSet::print(std::ostream& os) const {
  os << "Constraint set " << name() << ", contains" << incindent;
  for (Constraints_t::const_iterator itConstraint = constraints_.begin();
       itConstraint != constraints_.end(); itConstraint++) {
    os << iendl << **itConstraint;
  }
  return os << decindent;
}

ConfigProjectorPtr_t ConstraintSet::_configProj() const {
  return HPP_STATIC_PTR_CAST(
      ConfigProjector, constraints_[configProjI_ >= 0 ? configProjI_ : 0]);
}

template <class Archive>
void ConstraintSet::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<Constraint>(*this));
  ar& BOOST_SERIALIZATION_NVP(constraints_);
  ar& BOOST_SERIALIZATION_NVP(configProjI_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ConstraintSet);
}  // namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::core::ConstraintSet)
