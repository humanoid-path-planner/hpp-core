// Copyright (c) 2019, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_CORE_PLUGIN_HH
#define HPP_CORE_PLUGIN_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <string>

namespace hpp {
namespace core {
/// \addtogroup hpp_core_plugin Plugins
/// \{

/// Plugin mechanism to declare new features in ProblemSolver class
class ProblemSolverPlugin {
 public:
  const std::string& name() const { return name_; }

  const std::string& version() const { return version_; }

  bool initialize(ProblemSolverPtr_t ps) {
    if (initialized_) return true;
    initialized_ = impl_initialize(ps);
    return initialized_;
  }

  virtual ~ProblemSolverPlugin() {}

 protected:
  virtual bool impl_initialize(ProblemSolverPtr_t ps) = 0;

  ProblemSolverPlugin(const std::string& name, const std::string& version)
      : name_(name), version_(version), initialized_(false) {}

 private:
  std::string name_, version_;
  bool initialized_;
};  // class ProblemSolver

/// To create a plugin, create a class that derives from
/// hpp::core::ProblemSolverPlugin and call this macro in the root namespace.
/// To load the plugin, use
/// \code
/// std::string filename;
/// hpp::core::ProblemSolver ps*;
/// hpp::core::plugin::loadPlugin (filename, ps);
/// \endcode
#define HPP_CORE_DEFINE_PLUGIN(PluginClassName)                   \
  extern "C" {                                                    \
  ::hpp::core::ProblemSolverPlugin* createProblemSolverPlugin() { \
    return new PluginClassName();                                 \
  }                                                               \
  }

namespace plugin {
/// Find the absolute path to a library named name.
/// \param name
/// \return name if it is an absolute path.
///         Otherwise, for each path in HPP_PLUGIN_DIRS environment variable,
///         look for path/hppPlugins/name.
///         Otherwise, for each path in LD_LIBRARY_PATH environment variable,
///         look for path/hppPlugins/name.
/// \throw std::invalid_argument if not valid file found.
std::string findPluginLibrary(const std::string& name);

/// Load a plugin into ProblemSolver
/// 1. Call \ref findPluginLibrary
/// 2. Call dlopen and handle errors
/// 3. ProblemSolverPlugin::initialize with \c ps
bool loadPlugin(const std::string& lib, ProblemSolverPtr_t ps);
}  // namespace plugin

/// \}

}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PLUGIN_HH
