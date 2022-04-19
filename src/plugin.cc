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

#include <dlfcn.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <hpp/core/plugin.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <pinocchio/utils/file-explorer.hpp>

namespace fs = boost::filesystem;

namespace hpp {
namespace core {
namespace plugin {
std::string findPluginLibrary(const std::string& name) {
  if (fs::path(name).is_absolute()) return name;
  std::vector<std::string> ldpaths =
      ::pinocchio::extractPathFromEnvVar("LD_LIBRARY_PATH", ":");
  for (std::size_t i = 0; i < ldpaths.size(); ++i) {
    fs::path lib(ldpaths[i]);
    lib /= "hppPlugins";
    lib /= name;
    if (fs::is_regular_file(lib)) return lib.native();
  }
  HPP_THROW(std::invalid_argument, "Could not find plugin "
                                       << name
                                       << ". Check your LD_LIBRARY_PATH.");
}

bool loadPlugin(const std::string& lib, ProblemSolverPtr_t ps) {
  typedef ::hpp::core::ProblemSolverPlugin* (*PluginFunction_t)();

  // Clear old errors
  const char* error = dlerror();
  // void* library = dlopen(lib.c_str(), RTLD_NOW|RTLD_GLOBAL);
  void* library = dlopen(lib.c_str(), RTLD_NOW);
  error = dlerror();
  if (error != NULL) {
    throw std::runtime_error("Error loading library " + lib + ": " + error);
    return false;
  }
  if (library == NULL) {
    // uncaught error ?
    return false;
  }

  PluginFunction_t function = reinterpret_cast<PluginFunction_t>(
      dlsym(library, "createProblemSolverPlugin"));
  error = dlerror();
  if (error != NULL) {
    throw std::runtime_error("Error loading library " + lib + ": " + error);
    return false;
  }
  if (function == NULL) {
    throw std::runtime_error(
        "Symbol createProblemSolverPlugin of "
        "(correctly loaded) library " +
        lib + " is NULL.");
    return false;
  }

  ProblemSolverPlugin* plugin = function();
  bool success = plugin->initialize(ps);
  delete plugin;

  // I don't think we should do that because the symbols should not be
  // removed... dlclose (library);

  return success;
}
}  // namespace plugin
}  // namespace core
}  // namespace hpp
