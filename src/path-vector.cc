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
#include <hpp/core/path-vector.hh>
#include <hpp/util/indent.hh>
#include <hpp/util/serialization.hh>
#include <stdexcept>

namespace hpp {
namespace core {

std::size_t PathVector::rankAtParam(const value_type& param,
                                    value_type& localParam) const {
  assert(!timeParameterization());
  if (paths_.empty()) throw std::runtime_error("PathVector is empty.");
  std::size_t res = 0;
  localParam = param;
  bool finished = false;

  while (res + 1 < paths_.size() && !finished) {
    if (localParam > paths_[res]->length()) {
      localParam -= paths_[res]->length();
      res++;
    } else {
      finished = true;
    }
  }
  if (localParam > paths_[res]->length()) {
    if (res != paths_.size() - 1) {
      throw std::runtime_error("localparam out of range.");
    }
    localParam = paths_[res]->timeRange().second;
  } else {
    localParam += paths_[res]->timeRange().first;
  }
  assert(localParam >= paths_[res]->timeRange().first -
                           std::numeric_limits<float>::epsilon());
  assert(localParam <= paths_[res]->timeRange().second +
                           std::numeric_limits<float>::epsilon());
  return res;
}

void PathVector::appendPath(const PathPtr_t& path) {
  assert(!timeParameterization());
  paths_.push_back(path);
  interval_t tr = timeRange();
  tr.second += path->length();
  timeRange(tr);
}

PathPtr_t PathVector::pathAtRank(std::size_t rank) const {
  PathPtr_t copy;
  if (constraints()) {
    if (paths_[rank]->constraints()) {
      throw std::runtime_error(
          "Attempt to extract a path from a path vector where both "
          "are subject to constraints. This is not supported.");
    } else {
      ConstraintPtr_t constraintCopy(constraints()->copy());
      HPP_STATIC_CAST_REF_CHECK(ConstraintSet, *constraintCopy);
      copy = paths_[rank]->copy(
          HPP_STATIC_PTR_CAST(ConstraintSet, constraintCopy));
    }
  } else {
    copy = paths_[rank]->copy();
  }
  return copy;
}

void PathVector::concatenate(const PathVector& path) {
  for (std::size_t i = 0; i < path.numberPaths(); ++i) {
    appendPath(path.pathAtRank(i)->copy());
  }
}

void PathVector::concatenate(const PathVectorPtr_t& path) {
  for (std::size_t i = 0; i < path->numberPaths(); ++i) {
    appendPath(path->pathAtRank(i)->copy());
  }
}

void PathVector::flatten(PathVectorPtr_t p) const {
  assert(!timeParameterization());
  for (std::size_t i = 0; i < numberPaths(); ++i) {
    PathPtr_t path = pathAtRank(i);
    PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST(PathVector, path);
    if (pv)
      pv->flatten(p);
    else
      p->appendPath(path);
  }
}

bool PathVector::impl_compute(ConfigurationOut_t result,
                              value_type param) const {
  assert(!timeParameterization());
  // Find direct path in vector corresponding to parameter.
  size_t rank;
  value_type localParam;
  rank = rankAtParam(param, localParam);

  PathPtr_t subpath = paths_[rank];
  return (*subpath)(result, localParam);
}

void PathVector::impl_derivative(vectorOut_t result, const value_type& param,
                                 size_type order) const {
  assert(!timeParameterization());
  // Find direct path in vector corresponding to parameter.
  size_t rank;
  value_type localParam;
  rank = rankAtParam(param, localParam);

  PathPtr_t subpath = paths_[rank];
  subpath->derivative(result, localParam, order);
}

inline const value_type& Iinit(const interval_t& I, bool reverse) {
  return (reverse ? I.second : I.first);
}
inline const value_type& Iend(const interval_t& I, bool reverse) {
  return (reverse ? I.first : I.second);
}

void PathVector::impl_velocityBound(vectorOut_t bound, const value_type& param0,
                                    const value_type& param1) const {
  assert(!timeParameterization());

  bool reversed = param0 > param1 ? true : false;
  value_type localtinit, localtend;
  std::size_t iinit = rankAtParam(param0, localtinit),
              iend = rankAtParam(param1, localtend);
  if (iinit == iend) {
    paths_[iinit]->velocityBound(bound, localtinit, localtend);
  } else {
    paths_[iinit]->velocityBound(bound, localtinit,
                                 Iend(paths_[iinit]->timeRange(), reversed));
    int one = (reversed ? -1 : 1);
    vector_t localbound(vector_t::Zero(bound.size()));
    for (std::size_t i = iinit + one; (reversed && i > iend) || i < iend;
         i += one) {
      paths_[i]->velocityBound(localbound, paths_[i]->timeRange().first,
                               paths_[i]->timeRange().second);
      bound = bound.cwiseMax(localbound);
    }
    paths_[iend]->velocityBound(
        localbound, Iinit(paths_[iend]->timeRange(), reversed), localtend);
    bound = bound.cwiseMax(localbound);
  }
}

PathPtr_t PathVector::impl_extract(const interval_t& subInterval) const {
  assert(!timeParameterization());
  PathVectorPtr_t path = create(outputSize(), outputDerivativeSize());
  bool reversed = subInterval.first > subInterval.second ? true : false;

  value_type localtinit, localtend;
  std::size_t iinit = rankAtParam(subInterval.first, localtinit),
              iend = rankAtParam(subInterval.second, localtend);
  if (iinit == iend) {
    path->appendPath(paths_[iinit]->extract(localtinit, localtend));
  } else {
    path->appendPath(paths_[iinit]->extract(
        localtinit, Iend(paths_[iinit]->timeRange(), reversed)));
    int one = (reversed ? -1 : 1);
    for (std::size_t i = iinit + one; (reversed && i > iend) || i < iend;
         i += one)
      path->appendPath(reversed ? paths_[i]->reverse() : paths_[i]);
    path->appendPath(paths_[iend]->extract(
        Iinit(paths_[iend]->timeRange(), reversed), localtend));
  }
  return path;
}

PathPtr_t PathVector::reverse() const {
  assert(!timeParameterization());
  PathVectorPtr_t ret = create(outputSize(), outputDerivativeSize());
  std::for_each(paths_.rbegin(), paths_.rend(), [&ret](const PathPtr_t& path) {
    ret->appendPath(path->reverse());
  });
  return ret;
}

std::ostream& PathVector::print(std::ostream& os) const {
  Path::print(os << "PathVector:") << incendl;
  for (Paths_t::const_iterator itPath = paths_.begin(); itPath != paths_.end();
       ++itPath) {
    os << (**itPath) << iendl;
  }
  return os << decindent;
}

template <class Archive>
void PathVector::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<Path>(*this));
  ar& BOOST_SERIALIZATION_NVP(paths_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(PathVector);
}  //   namespace core
}  // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::core::PathVector)
