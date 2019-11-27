// Copyright (c) 2019, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/plugin.hh>

#include <dlfcn.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <pinocchio/utils/file-explorer.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

namespace fs = boost::filesystem;

namespace hpp {
  namespace core {
    namespace plugin {
      std::string findPluginLibrary (const std::string& name)
      {
        if (fs::path(name).is_absolute ()) return name;
        std::vector<std::string> ldpaths =
          ::pinocchio::extractPathFromEnvVar ("LD_LIBRARY_PATH", ":");
        for (std::size_t i = 0; i < ldpaths.size(); ++i) {
          fs::path lib (ldpaths[i]);
          lib /= "hppPlugins";
          lib /= name;
          if (fs::is_regular_file (lib)) return lib.native();
        }
        HPP_THROW (std::invalid_argument, "Could not find plugin " << name
            << ". Check your LD_LIBRARY_PATH.");
      }

      bool loadPlugin (const std::string& lib, ProblemSolver* ps)
      {
        typedef ::hpp::core::ProblemSolverPlugin* (*PluginFunction_t) ();

        // Clear old errors
        const char* error = dlerror ();
        //void* library = dlopen(lib.c_str(), RTLD_NOW|RTLD_GLOBAL);
        void* library = dlopen(lib.c_str(), RTLD_NOW);
        error = dlerror ();
        if (error != NULL) {
          throw std::runtime_error ("Error loading library " + lib + ": " +
              error);
          return false;
        }
        if (library == NULL) {
          // uncaught error ?
          return false;
        }

        PluginFunction_t function = reinterpret_cast<PluginFunction_t>(dlsym(library, "createProblemSolverPlugin"));
        error = dlerror ();
        if (error != NULL) {
          throw std::runtime_error ("Error loading library " + lib + ": " +
              error);
          return false;
        }
        if (function == NULL) {
          throw std::runtime_error ("Symbol createProblemSolverPlugin of "
              "(correctly loaded) library " + lib + " is NULL.");
          return false;
        }

        ProblemSolverPlugin* plugin = function();
        bool success = plugin->initialize (ps);
        delete plugin;

        // I don't think we should do that because the symbols should not be removed...
        // dlclose (library);

        return success;
      }
    } // namespace plugin
  } // namespace core
} // namespace hpp
