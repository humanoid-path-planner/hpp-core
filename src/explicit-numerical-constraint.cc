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

#include <hpp/model/device.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/core/comparison-type.hh>
#include <hpp/core/explicit-numerical-constraint.hh>

namespace hpp {
  namespace core {
    void complement (size_type size, const SizeIntervals_t& intervals,
		     SizeIntervals_t& result)
    {
      std::vector <bool> unionOfIntervals (size+1, false);
      for (SizeIntervals_t::const_iterator it = intervals.begin ();
	   it != intervals.end (); ++it) {
	for (size_type i=it->first; i < it->first + it->second; ++i) {
	  unionOfIntervals [i] = true;
	}
      }
      unionOfIntervals [size] = true;
      unsigned int state = 0;
      SizeInterval_t interval;
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

    HPP_PREDEF_CLASS (ImplicitFunction);
    typedef boost::shared_ptr <ImplicitFunction> ImplicitFunctionPtr_t;
    class ImplicitFunction : public DifferentiableFunction
    {
    public:
      static ImplicitFunctionPtr_t create
      (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
       const SizeIntervals_t& outputConf, const SizeIntervals_t& outputVelocity)
      {
	ImplicitFunction* ptr = new ImplicitFunction
	  (robot, function, outputConf, outputVelocity);
	return ImplicitFunctionPtr_t (ptr);
      }
      void solve (ConfigurationOut_t configuration, vectorIn_t rhs)
      {
	assert (rhs.size () == output_.size ());
	size_type index = 0;
	for (SizeIntervals_t::const_iterator it = inputConfIntervals_.begin ();
	     it != inputConfIntervals_.end (); ++it) {
	  input_.segment (index, it->second) =
	    configuration.segment (it->first, it->second);
	  index += it->second;
	}
	(*inputToOutput_) (output_, input_);
	index = 0;
	for (SizeIntervals_t::const_iterator it = outputConfIntervals_.begin ();
	     it != outputConfIntervals_.end (); ++it) {
	  configuration.segment (it->first, it->second) =
	    output_.segment (index, it->second) +
	    rhs.segment (index, it->second);
	  index += it->second;
	}
      }

    protected:
      ImplicitFunction (const DevicePtr_t& robot,
			const DifferentiableFunctionPtr_t& function,
			const SizeIntervals_t& outputConf,
			const SizeIntervals_t& outputVelocity)
	: DifferentiableFunction (robot->configSize (), robot->numberDof (),
				  function->outputSize (),
				  function->outputDerivativeSize ()),
	  robot_ (robot), inputToOutput_ (function), inputConfIntervals_ (),
	  inputDerivIntervals_ (), outputConfIntervals_ (outputConf),
	  outputDerivIntervals_ (outputVelocity)
      {
	// Check input consistency
	// Each configuration variable is either input or output
	assert (function->inputSize () + function->outputSize () ==
		robot->configSize ());
	// Each velocity variable is either input or output
	assert (function->inputDerivativeSize () +
		function->outputDerivativeSize () == robot->numberDof ());
	input_.resize (function->inputSize ());
	output_.resize (function->outputSize ());
	J_.resize (function->outputDerivativeSize (),
		   function->inputDerivativeSize ());
	size_type size = 0;
	// Sum of configuration output interval sizes equal function output size
	for (SizeIntervals_t::const_iterator it = outputConf.begin ();
	     it != outputConf.end (); ++it) {
	  size += it->second;
	}
	assert (size == function->outputSize ());
	// Sum of velocity output interval sizes equal function output
	// derivative size
	size = 0;
	for (SizeIntervals_t::const_iterator it = outputVelocity.begin ();
	     it != outputVelocity.end (); ++it) {
	  size += it->second;
	}
	assert (size == function->outputDerivativeSize ());
	// Conpute input intervals
	complement (robot->configSize (), outputConfIntervals_,
		    inputConfIntervals_);
	complement (robot->numberDof (), outputDerivIntervals_,
		    inputDerivIntervals_);
      }

      void impl_compute (vectorOut_t result, vectorIn_t argument) const
      {
	size_type index = 0;
	for (SizeIntervals_t::const_iterator it = outputConfIntervals_.begin ();
	     it != outputConfIntervals_.end (); ++it) {
	  result.segment (index, it->second) =
	    argument.segment (it->first, it->second);
	  index += it->second;
	}
	index = 0;
	for (SizeIntervals_t::const_iterator it = inputConfIntervals_.begin ();
	     it != inputConfIntervals_.end (); ++it) {
	  input_.segment (index, it->second) =
	    argument.segment (it->first, it->second);
	  index += it->second;
	}
	(*inputToOutput_) (output_, input_);
	result -= output_;
      }

      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const
      {
	jacobian.setZero ();
	size_type row = 0;
	for (SizeIntervals_t::const_iterator it=outputDerivIntervals_.begin ();
	     it != outputDerivIntervals_.end (); ++it) {
	  for (size_type col=it->first; col < it->first + it->second; ++col) {
	    jacobian (row, col) = 1;
	    ++row;
	  }
	}

	size_type index = 0;
	for (SizeIntervals_t::const_iterator it = inputConfIntervals_.begin ();
	     it != inputConfIntervals_.end (); ++it) {
	  input_.segment (index, it->second) =
	    arg.segment (it->first, it->second);
	  index += it->second;
	}
	inputToOutput_->jacobian (J_, input_);
	size_type col=0;
	size_type nbRows = inputToOutput_->outputDerivativeSize ();
	for (SizeIntervals_t::const_iterator it = inputDerivIntervals_.begin ();
	     it != inputDerivIntervals_.end (); ++it) {
	  jacobian.block (0, it->first, nbRows, it->second) =
	    - J_.block (0, col, nbRows, it->second);
	  col += it->second;
	}
      }

    private:
      DevicePtr_t robot_;
      DifferentiableFunctionPtr_t inputToOutput_;
      SizeIntervals_t inputConfIntervals_;
      SizeIntervals_t inputDerivIntervals_;
      SizeIntervals_t outputConfIntervals_;
      SizeIntervals_t outputDerivIntervals_;
      mutable vector_t input_;
      mutable vector_t output_;
      // Jacobian of explicit function
      mutable matrix_t J_;
    }; // class ImplicitFunction

    ExplicitNumericalConstraintPtr_t ExplicitNumericalConstraint::create
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const SizeIntervals_t& outputConf,
     const SizeIntervals_t& outputVelocity)
    {
      ExplicitNumericalConstraint* ptr = new ExplicitNumericalConstraint
	(robot, function, outputConf, outputVelocity);
      ExplicitNumericalConstraintPtr_t shPtr (ptr);
      ExplicitNumericalConstraintWkPtr_t wkPtr (shPtr);
      ptr->init (wkPtr);
      return shPtr;
    }

    ExplicitNumericalConstraintPtr_t ExplicitNumericalConstraint::create
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const SizeIntervals_t& outputConf,
     const SizeIntervals_t& outputVelocity, vectorIn_t rhs)
    {
      ExplicitNumericalConstraint* ptr = new ExplicitNumericalConstraint
	(robot, function, outputConf, outputVelocity, rhs);
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

    EquationPtr_t ExplicitNumericalConstraint::copy () const
    {
      return createCopy (weak_.lock ());
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const SizeIntervals_t& outputConf,
     const SizeIntervals_t& outputVelocity) :
      NumericalConstraint (ImplicitFunction::create
			   (robot, function, outputConf, outputVelocity),
			   Equality::create ()), outputConf_ (outputConf),
      outputVelocity_ (outputVelocity)
    {
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
     const SizeIntervals_t& outputConf,
     const SizeIntervals_t& outputVelocity, vectorIn_t rhs) :
      NumericalConstraint (ImplicitFunction::create
			   (robot, function, outputConf, outputVelocity),
			   Equality::create (), rhs), outputConf_ (outputConf),
      outputVelocity_ (outputVelocity)
    {
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const DifferentiableFunctionPtr_t& implicitConstraint,
     const SizeIntervals_t& outputConf,
     const SizeIntervals_t& outputVelocity) :
      NumericalConstraint (implicitConstraint, EqualToZero::create ()),
      outputConf_ (outputConf), outputVelocity_ (outputVelocity)
    {
    }

    ExplicitNumericalConstraint::ExplicitNumericalConstraint
    (const ExplicitNumericalConstraint& other) :
      NumericalConstraint (other), inputToOutput_ (other.inputToOutput_),
      outputConf_ (other.outputConf_), outputVelocity_ (other.outputVelocity_)
    {
    }

    void ExplicitNumericalConstraint::solve (ConfigurationOut_t configuration)
    {
      HPP_STATIC_CAST_REF_CHECK (ImplicitFunction, *(functionPtr ()));
      HPP_STATIC_PTR_CAST (ImplicitFunction, functionPtr ())->solve
	(configuration, rightHandSide ());
    }


  } // namespace core
} // namespace hpp
