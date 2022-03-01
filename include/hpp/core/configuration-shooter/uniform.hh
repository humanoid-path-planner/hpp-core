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

#ifndef HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH
# define HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH

# include <sstream>

# include <hpp/pinocchio/device.hh>

# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {
      /// \addtogroup configuration_sampling
      /// \{

      /// Uniformly sample with bounds of degrees of freedom.
      class HPP_CORE_DLLAPI Uniform : public ConfigurationShooter
      {
      public:
        static UniformPtr_t create (const DevicePtr_t& robot)
        {
          Uniform* ptr = new Uniform (robot);
          UniformPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

        void sampleExtraDOF(bool sampleExtraDOF){
            sampleExtraDOF_=sampleExtraDOF;
        }

      protected:
        /// Uniformly sample configuration space
        ///
        /// Note that translation joints have to be bounded.
        Uniform (const DevicePtr_t& robot) : robot_ (robot),sampleExtraDOF_(true)
        {
        }
        void init (const UniformPtr_t& self)
        {
          ConfigurationShooter::init (self);
          weak_ = self;
        }

        virtual void impl_shoot (Configuration_t& q) const;
      private:
        DevicePtr_t robot_;
        bool sampleExtraDOF_;
        UniformWkPtr_t weak_;
      }; // class Uniform
      /// \}
    } // namespace configurationShooter
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_CONFIGURATION_SHOOTER_UNIFORM_HH
