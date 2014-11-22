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

#ifndef HPP_CORE_DIFFERENTIABLE_FUNCTION_HH
# define HPP_CORE_DIFFERENTIABLE_FUNCTION_HH

# include <algorithm>
# include <boost/algorithm/string/replace.hpp>
# include <boost/assign/list_of.hpp>
# include <roboptim/core/indent.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// Differentiable function
    class HPP_CORE_DLLAPI DifferentiableFunction
    {
    public:
      typedef std::pair <size_type, size_type> Interval_t;
      typedef std::vector <Interval_t> Intervals_t;

      virtual ~DifferentiableFunction () {}
      /// Evaluate the function at a given parameter.
      ///
      /// \note parameters should be of the correct size.
      void operator () (vectorOut_t result,
			vectorIn_t argument) const
      {
	assert (result.size () == outputSize ());
	assert (argument.size () == inputSize ());
	impl_compute (result, argument);
      }
      /// Computes the jacobian.
      ///
      /// \retval jacobian jacobian will be stored in this argument
      /// \param argument point at which the jacobian will be computed
      void jacobian (matrixOut_t jacobian, vectorIn_t argument) const
      {
	assert (argument.size () == inputSize ());
	assert (jacobian.rows () == outputSize ());
	assert (jacobian.cols () == inputDerivativeSize ());
	impl_jacobian (jacobian, argument);
        for (size_t i = 0; i < passiveDofs_.size(); i++)
          jacobian.middleCols (passiveDofs_[i].first, passiveDofs_[i].second).setZero ();
      }

      /// Get dimension of input vector
      size_type inputSize () const
      {
	return inputSize_;
      }
      /// Get dimension of input derivative vector
      ///
      /// The dimension of configuration vectors might differ from the dimension
      /// of velocity vectors since some joints are represented by non minimal
      /// size vectors: e.g. quaternion for SO(3)
      size_type inputDerivativeSize () const
      {
	return inputDerivativeSize_;
      }
      /// Get dimension of output vector
      size_type  outputSize () const
      {
	return outputSize_;
      }
      /// \brief Get function name.
      ///
      /// \return Function name.
      const std::string& name () const
      {
	return name_;
      }

      /// Display object in a stream
      virtual std::ostream& print (std::ostream& o) const
      {
	if (this->name ().empty ())
	  return o << "Differentiable function";

	std::stringstream ss;
	ss << std::endl;
	char fill = o.fill (' ');
	ss << std::setw ((int)roboptim::indent (o))
	   << ""
	   << std::setfill (fill);
	std::string name = this->name ();
	boost::algorithm::replace_all (name, "\n", ss.str ());

	return o << name << " (differentiable function)";
      }

      /// Check whether this function is parametric.
      /// \return True if parametric.
      bool isParametric () const
      {
        return isParametric_;
      }

      /// Make the function parametric or non-parametric.
      /// \param value True if you want a parametric projector.
      /// \note When change from true to false, the level set parameters of any
      /// ConfigProjector containing the function should be recomputed using
      /// ConfigProjector::offset.
      void isParametric (const bool& value)
      {
        isParametric_ = value;
      }

      /// Set passive DOFs. Passive DOF cannot be modified by this function.
      /// Corresponding columns of the jacobian are set to zero.
      const Intervals_t& passiveDofs (std::vector <size_type> dofs)
      {
        passiveDofs_.clear ();
        if (dofs.size () == 0) return passiveDofs_;
        std::sort (dofs.begin (), dofs.end ());
        std::vector <size_type>::iterator it =
          std::unique (dofs.begin (), dofs.end ());
        dofs.resize (std::distance (dofs.begin (), it));
        dofs.push_back (inputDerivativeSize_ + 1);
        size_type intStart = dofs[0], intEnd = dofs[0];
        for (size_t i = 1; i < dofs.size (); i++) {
          intEnd ++;
          if (intEnd == dofs[i]) {
            continue;
          } else {
            passiveDofs_.push_back (Interval_t (intStart, intEnd - intStart));
            intStart = intEnd = dofs[i];
          }
        }
        return passiveDofs_;
      }

    protected:
      /// \brief Concrete class constructor should call this constructor.
      ///
      /// \param inputSize function arity
      /// \param outputSize result size
      /// \param name function's name
      DifferentiableFunction (size_type inputSize,
			      size_type inputDerivativeSize,
			      size_type outputSize,
			      std::string name = std::string ()) :
	inputSize_ (inputSize), inputDerivativeSize_ (inputDerivativeSize),
	outputSize_ (outputSize), isParametric_ (false),
       passiveDofs_ (0), name_ (name)
      {
      }

      /// User implementation of function evaluation
      virtual void impl_compute (vectorOut_t result,
				 vectorIn_t argument) const = 0;

      virtual void impl_jacobian (matrixOut_t jacobian,
				  vectorIn_t arg) const = 0;

    private:
      /// Dimension of input vector.
      size_type inputSize_;
      /// Dimension of input derivative
      size_type inputDerivativeSize_;
      /// Dimension of output vector
      size_type outputSize_;
      /// Whether this function is parametric
      bool isParametric_;
      /// Intervals of passive dofs
      Intervals_t passiveDofs_;
      std::string name_;
    }; // class DifferentiableFunction
    inline std::ostream&
    operator<< (std::ostream& os, const DifferentiableFunction& f)
    {
      return f.print (os);
    }
  } // namespace core
} // namespace hpp


#endif // HPP_CORE_DIFFERENTIABLE_FUNCTION_HH
