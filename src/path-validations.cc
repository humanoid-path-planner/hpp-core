//
// Copyright (c) 2017 CNRS
// Authors: Diane Bury
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

#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
namespace core {

PathValidationsPtr_t PathValidations::create() {
  PathValidations* ptr = new PathValidations();
  return PathValidationsPtr_t(ptr);
}

void PathValidations::addPathValidation(
    const PathValidationPtr_t& pathValidation) {
  validations_.push_back(pathValidation);
}

bool PathValidations::validate(const PathPtr_t& path, bool reverse,
                               PathPtr_t& validPart,
                               PathValidationReportPtr_t& validationReport) {
  PathPtr_t tempPath = path;
  PathPtr_t tempValidPart;
  PathValidationReportPtr_t tempValidationReport;

  bool result = true;
  value_type lastValidTime = path->timeRange().second;
  value_type t = lastValidTime;

  for (std::vector<PathValidationPtr_t>::iterator it = validations_.begin();
       it != validations_.end(); ++it) {
    if ((*it)->validate(tempPath, reverse, tempValidPart,
                        tempValidationReport) == false) {
      t = tempValidationReport->getParameter();
      if (t < lastValidTime) {
        lastValidTime = t;
        tempPath = tempValidPart;
      }
      result = false;
    }
  }
  validPart = tempPath;
  validationReport->setParameter(lastValidTime);
  return result;
}

bool PathValidations::validate(ConfigurationIn_t q,
                               ValidationReportPtr_t& report) {
  for (auto pv : validations_) {
    bool res(pv->validate(q, report));
    if (!res) return false;
  }
  return true;
}

PathValidations::PathValidations() {}
}  // namespace core
}  // namespace hpp
