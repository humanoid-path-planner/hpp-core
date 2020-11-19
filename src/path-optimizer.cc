// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/core/path-optimizer.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/steering-method.hh>

namespace bpt = boost::posix_time;

namespace hpp {
  namespace core {
    PathOptimizer::PathOptimizer (const Problem& problem) :
      interrupt_ (false), problem_ (problem),
      maxIterations_ (std::numeric_limits <unsigned long int>::infinity ()),
      timeOut_ (std::numeric_limits <double>::infinity ())
    {
      monitor_.enabled = false;

      initFromParameters();
    }

    PathPtr_t PathOptimizer::steer (ConfigurationIn_t q1,
        ConfigurationIn_t q2) const
    {
      PathPtr_t dp = (*problem().steeringMethod())(q1,q2);
      if (dp) {
        if (!problem().pathProjector()) return dp;
        PathPtr_t pp;
        if (problem().pathProjector()->apply (dp, pp))
          return pp;
      }
      return PathPtr_t ();
    }

    void PathOptimizer::monitorExecution()
    {
      interrupt_ = false;
      monitor_.enabled = true;
      monitor_.iteration = 0;
      monitor_.timeStart = bpt::microsec_clock::universal_time();
    }

    bool PathOptimizer::shouldStop() const
    {
      if (interrupt_) return true;
      if (!monitor_.enabled) return false;
      if (monitor_.iteration >= maxIterations_) return true;

      bpt::ptime timeStop(bpt::microsec_clock::universal_time());
      if(static_cast<value_type>((timeStop - monitor_.timeStart)
            .total_milliseconds()) > timeOut_*1e3)
        return true;
      return false;
    }

    void PathOptimizer::initFromParameters ()
    {
      maxIterations_ = problem().getParameter ("PathOptimizer/maxIterations").intValue();
      timeOut_ = problem().getParameter ("PathOptimizer/timeOut").floatValue();
    }

    void PathOptimizer::maxIterations (const unsigned long int& n)
    {
      maxIterations_ = n;
    }

    void PathOptimizer::timeOut(const double& timeOut)
    {
      timeOut_ = timeOut;
    }

    // ----------- Declare parameters ------------------------------------- //

    HPP_START_PARAMETER_DECLARATION(PathOptimizer)
    Problem::declareParameter (ParameterDescription (Parameter::INT,
          "PathOptimizer/maxIterations",
          "Maximal number of iterations.",
          Parameter (std::numeric_limits <size_type>::max ())));
    Problem::declareParameter (ParameterDescription (Parameter::FLOAT,
          "PathOptimizer/timeOut",
          "Duration in seconds above which execution will stop."
          "The iteration at the moment the duration is elapsed will be completed.",
          Parameter (std::numeric_limits <double>::infinity ())));
    HPP_END_PARAMETER_DECLARATION(PathOptimizer)
  } // namespace core
} // namespace hpp

