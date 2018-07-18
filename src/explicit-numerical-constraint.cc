// Copyright (c) 2015, LAAS-CNRS
// Authors: Florent Lamiraux
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

#include <hpp/pinocchio/device.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include "../src/implicit-function.hh"

namespace hpp {
  namespace core {
    typedef constraints::Implicit Implicit;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    void complement (size_type size, const segments_t& intervals,
		     segments_t& result)
    {
      std::vector <bool> unionOfIntervals (size+1, false);
      for (segments_t::const_iterator it = intervals.begin ();
	   it != intervals.end (); ++it) {
	for (size_type i=it->first; i < it->first + it->second; ++i) {
	  unionOfIntervals [i] = true;
	}
      }
      unionOfIntervals [size] = true;
      unsigned int state = 0;
      segment_t interval;
      for (size_type i=0; i <= (size_type) unionOfIntervals.size (); ++i) {
	if ((state == 0) && (unionOfIntervals [i] == false)) {
	  // start a new interval
	  state = 1;
	  interval.first = i;
	} else if ((state == 1) && unionOfIntervals [i] == true) {
	  // finish current interval
	  state = 0;
	  interval.second = i - interval.first;
	  result.push_back (interval);
	}
      }
    }

    inline ComparisonTypes_t defaultCompTypes (
        const segments_t& outputVelocity,
        const ComparisonTypes_t& comp)
    {
      if (comp.size() == 0) {
        size_type n = Eigen::BlockIndex::cardinal (outputVelocity);
        if (n > 0)
          return ComparisonTypes_t(n, constraints::EqualToZero);
      }
      return comp;
    }

    ExplicitNumericalConstraintPtr_t ExplicitNumericalConstraint::create
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const DifferentiableFunctionPtr_t& g,
     const DifferentiableFunctionPtr_t& ginv,
     const segments_t& inputConf,
     const segments_t& inputVelocity,
     const segments_t& outputConf,
     const segments_t& outputVelocity,
     const ComparisonTypes_t& comp)
    {
      ExplicitNumericalConstraint* ptr = new ExplicitNumericalConstraint
	(robot, function, g, ginv, inputConf, inputVelocity, outputConf, outputVelocity,
         defaultCompTypes(outputVelocity,comp));
      ExplicitNumericalConstraintPtr_t shPtr (ptr);
      ExplicitNumericalConstraintWkPtr_t wkPtr (shPtr);
      ptr->init (wkPtr);
      return shPtr;
    }

    ExplicitNumericalConstraintPtr_t ExplicitNumericalConstraint::create
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const segments_t& inputConf,
     const segments_t& inputVelocity,
     const segments_t& outputConf,
     const segments_t& outputVelocity,
     const ComparisonTypes_t& comp)
    {
      ExplicitNumericalConstraint* ptr = new ExplicitNumericalConstraint
	(robot, function, inputConf, inputVelocity, outputConf, outputVelocity,
         defaultCompTypes(outputVelocity,comp));
      ExplicitNumericalConstraintPtr_t shPtr (ptr);
      ExplicitNumericalConstraintWkPtr_t wkPtr (shPtr);
      ptr->init (wkPtr);
      return shPtr;
    }

    ExplicitNumericalConstraintPtr_t ExplicitNumericalConstraint::createCopy
    (const ExplicitNumericalConstraintPtr_t& other)
    {
      ExplicitNumericalConstraint* ptr = new ExplicitNumericalConstraint
	(*other);
      ExplicitNumericalConstraintPtr_t shPtr (ptr);
      ExplicitNumericalConstraintWkPtr_t wkPtr (shPtr);
      ptr->init (wkPtr);
      return shPtr;
    }

    ImplicitPtr_t ExplicitNumericalConstraint::copy () const
    {
      return createCopy (weak_.lock ());
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& explicitFunction,
     const DifferentiableFunctionPtr_t& g,
     const DifferentiableFunctionPtr_t& ginv,
     const segments_t& inputConf,
     const segments_t& inputVelocity,
     const segments_t& outputConf,
     const segments_t& outputVelocity,
     const ComparisonTypes_t& comp) :
      Implicit (GenericImplicitFunction::create
                (robot, explicitFunction, g, inputConf, inputVelocity,
                 outputConf, outputVelocity),
                comp),
      inputToOutput_ (explicitFunction), g_ (g), ginv_ (ginv),
      inputConf_ (inputConf),
      inputVelocity_ (inputVelocity),
      outputConf_ (outputConf),
      outputVelocity_ (outputVelocity)
    {
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& explicitFunction,
     const segments_t& inputConf,
     const segments_t& inputVelocity,
     const segments_t& outputConf,
     const segments_t& outputVelocity,
     const ComparisonTypes_t& comp) :
      Implicit (BasicImplicitFunction::create
                (robot, explicitFunction, inputConf, inputVelocity,
                 outputConf, outputVelocity),
                comp),
      inputToOutput_ (explicitFunction),
      inputConf_ (inputConf),
      inputVelocity_ (inputVelocity),
      outputConf_ (outputConf),
      outputVelocity_ (outputVelocity)
    {
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const ExplicitNumericalConstraint& other) :
      Implicit (other), inputToOutput_ (other.inputToOutput_),
      inputConf_ (other.inputConf_), inputVelocity_ (other.inputVelocity_),
      outputConf_ (other.outputConf_), outputVelocity_ (other.outputVelocity_)
    {
    }
  } // namespace core
} // namespace hpp
