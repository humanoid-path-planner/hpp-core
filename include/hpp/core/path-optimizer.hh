//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_OPTIMIZER_HH
# define HPP_CORE_PATH_OPTIMIZER_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

# include <boost/date_time/posix_time/ptime.hpp>

namespace hpp {
  namespace core {
    /// \addtogroup path_optimization
    /// \{

    /// Abstraction of path optimizer
    ///
    class HPP_CORE_DLLAPI PathOptimizer
    {
    public:
      virtual ~PathOptimizer () {};

      /// Get problem
      const Problem& problem () const
      {
	return problem_;
      }

      /// Optimize path
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path) = 0;

      /// Interrupt path optimization
      void interrupt () { interrupt_ = true; }
      /// Set maximal number of iterations
      void maxIterations (const unsigned long int& n);
      /// set time out (in seconds)
      void timeOut(const double& timeOut);

    protected:
      /// Whether to interrupt computation
      /// Set to false at start of optimize method, set to true by method
      /// interrupt.
      bool interrupt_;

      PathOptimizer (const Problem& problem);

      PathPtr_t steer (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

      void monitorExecution();

      void endIteration() { ++monitor_.iteration; }

      bool shouldStop() const;

      void initFromParameters ();

    private:
      const Problem& problem_;

      /// Maximal number of iterations to solve a problem
      /// reaching this bound raises an exception.
      size_type maxIterations_;
      /// Time out (in seconds) before interrupting the planning
      double timeOut_;

      /// Information used to monitor the execution of the algorithm.
      /// This information is:
      /// \li initialized by \ref monitorExecution
      /// \li updated by \ref endIteration
      /// \li read by \ref shouldStop
      struct {
        bool enabled;
        size_type iteration;
        boost::posix_time::ptime timeStart;
      } monitor_;
    }; // class PathOptimizer;
    /// }
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZER_HH
