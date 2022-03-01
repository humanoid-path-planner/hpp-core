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


#include <hpp/core/config-validations.hh>
#include <hpp/core/validation-report.hh>

namespace hpp {
  namespace core {
    ConfigValidationsPtr_t ConfigValidations::create ()
    {
      ConfigValidations* ptr = new ConfigValidations;
      return ConfigValidationsPtr_t (ptr);
    }

    bool ConfigValidations::validate (const Configuration_t& config,
				      ValidationReportPtr_t& validationReport)
    {
      for (std::vector <ConfigValidationPtr_t>::iterator
       it = validations_.begin (); it != validations_.end (); ++it) {
	if ((*it)->validate (config, validationReport)
	    == false) {
	  return false;
	}
      }
      return true;
    }

    void ConfigValidations::add (const ConfigValidationPtr_t& configValidation)
    {
      validations_.push_back (configValidation);
    }

    size_type ConfigValidations::numberConfigValidations () const
    {
      return (size_type) validations_.size ();
    }
  } // namespace core
} // namespace hpp
