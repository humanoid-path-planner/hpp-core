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

#ifndef HPP_CORE_PLUGIN_HH
#define HPP_CORE_PLUGIN_HH

#include <string>

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup hpp_core_plugin Plugins
    /// \{

    /// Plugin mechanism to declare new features in ProblemSolver class
    class ProblemSolverPlugin {
      public:
        const std::string& name () const
        {
          return name_;
        }

        const std::string& version () const
        {
          return version_;
        }

        bool initialize (ProblemSolverPtr_t ps)
        {
          if (initialized_) return true;
          initialized_ = impl_initialize (ps);
          return initialized_;
        }

        virtual ~ProblemSolverPlugin () {}

      protected:
        virtual bool impl_initialize (ProblemSolverPtr_t ps) = 0;

        ProblemSolverPlugin (const std::string& name, const std::string& version)
          : name_ (name), version_ (version), initialized_ (false)
        {}

      private:
        std::string name_, version_;
        bool initialized_;
    }; // class ProblemSolver

    /// To create a plugin, create a class that derives from
    /// hpp::core::ProblemSolverPlugin and call this macro in the root namespace.
    /// To load the plugin, use
    /// \code
    /// std::string filename;
    /// hpp::core::ProblemSolver ps*;
    /// hpp::core::plugin::loadPlugin (filename, ps);
    /// \endcode
#define HPP_CORE_DEFINE_PLUGIN(PluginClassName)                                \
    extern "C" {                                                               \
      ::hpp::core::ProblemSolverPlugin* createProblemSolverPlugin ()           \
      {                                                                        \
        return new PluginClassName ();                                         \
      }                                                                        \
    }

    namespace plugin {
      /// Find the absolute path to a library named name.
      /// \param name
      /// \return name if it is an absolute path. Otherwise, for each path in
      ///         LD_LIBRARY_PATH environment variable, look for path/hppPlugins/name.
      /// \throw std::invalid_argument if not valid file found.
      std::string findPluginLibrary (const std::string& name);

      /// Load a plugin into ProblemSolver
      /// 1. Call \ref findPluginLibrary
      /// 2. Call dlopen and handle errors
      /// 3. ProblemSolverPlugin::initialize with \c ps
      bool loadPlugin (const std::string& lib, ProblemSolver* ps);
    } // namespace plugin

    /// \}

  } // namespace core
} // namespace hpp

#endif // HPP_CORE_PLUGIN_HH
