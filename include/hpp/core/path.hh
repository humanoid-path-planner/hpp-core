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

#ifndef HPP_CORE_PATH_HH
# define HPP_CORE_PATH_HH

# include <boost/concept_check.hpp>
# include <roboptim/trajectory/trajectory.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/constraint-set.hh>

namespace hpp {
  namespace core {
    /// Abstraction of paths: mapping from time to configuration space
    ///
    /// This class derives from roboptim::Trajectory as such Path could be
    /// used for numerical optimization. For that some unimplemented methods
    /// should be implemented.
    class HPP_CORE_DLLAPI Path : public roboptim::Trajectory <3>
    {
    public:
      typedef roboptim::Trajectory <3> parent_t;
      /// \name Construction, destruction, copy
      /// \{

      /// Destructor
      virtual ~Path () throw () {}

      virtual Path* clone () const throw (){
	throw std::runtime_error
	  ("Do not use this function, use copy instead.");
      }
      /// Return a shared pointer to a copy of this
      virtual PathPtr_t copy () const = 0;

      /// Static cast into a derived type
      template <class T> boost::shared_ptr<T> as (void)
      {
	BOOST_CONCEPT_ASSERT ((boost::Convertible <T*, Path*>));
	assert (HPP_DYNAMIC_PTR_CAST (T, weak_.lock ()));
	return HPP_STATIC_PTR_CAST (T, weak_.lock ());
      }

      /// Static cast into a derived type
      template <class T> boost::shared_ptr<const T> as (void) const
      {
	BOOST_CONCEPT_ASSERT ((boost::Convertible <T*, Path*>));
	assert (HPP_DYNAMIC_PTR_CAST (const T, weak_.lock ()));
	return HPP_STATIC_PTR_CAST (const T, weak_.lock ());
      }
	
      /// Extraction/Reversion of a sub-path
      /// \param subInterval interval of definition of the extract path
      /// If upper bound of subInterval is smaller than lower bound,
      /// result is reversed.
      virtual PathPtr_t extract (const interval_t& subInterval) const;
      /// \}

      /// \name Unimplemented methods from roboptim-trajectory
      /// \{

      /// Get the variation of a configuration with respect to parameter
      /// vector.
      /// \param t value \f$t\f$ in the definition interval.
      /// \return Jacobian:
      /// \f[\frac{\partial\Gamma_{\textbf{p}}(t)}{\partial\textbf{p}}\f]
      virtual jacobian_t variationConfigWrtParam (value_type) const throw ()
      {
	throw std::runtime_error ("Not implemented");
      }

      /// Get the variation of a derivative with respect to parameter vector.
      ///
      /// \param t value \f$t\f$ in the definition interval.
      /// \param order order \f$r\f$ of the derivative.
      /// \return jacobian
      /// \f[
      /// \frac{\partial}{\partial\textbf{p}}
      /// \left(\frac{d^r\Gamma_{\textbf{p}}}{dt^r}(t)\right)
      /// \f]
      virtual jacobian_t variationDerivWrtParam (value_type, size_type)
      const throw ()
      {
	throw std::runtime_error ("Not implemented");
      }

      /// \brief Get singular point at given rank.
      virtual value_type singularPointAtRank (size_type) const
      {
	throw std::runtime_error ("No singular point");
      }

      /// \brief Get left limit value of derivative at given singular point
      ///
      /// \param rank rank of the singular points.
      /// \param order order of derivation.
      /// \return Limit of the derivative at singular point
      /// for increasing parameter values.
      virtual vector_t derivBeforeSingularPoint (size_type,
						 size_type) const
      {
	throw std::runtime_error ("No singular point");
      }

      /// \brief Get right limit value of derivative at given singular point
      /// \param rank rank of the singular points.
      /// \param order order of derivation.
      /// \retval derivative Limit of the derivative at singular point for
      /// decreasing parameter values.
      virtual vector_t derivAfterSingularPoint (size_type, size_type)
	const
      {
	throw std::runtime_error ("No singular point");
      }

      virtual jacobian_t
      variationConfigWrtParam (StableTimePoint_t)
	const throw ()
      {
	throw std::runtime_error ("Not implemented");
      }

      virtual jacobian_t
      variationDerivWrtParam (StableTimePoint_t, size_type)
	const throw ()
      {
	throw std::runtime_error ("Not implemented");
      }

      virtual Path* resize (interval_t) const throw ()
      {
	throw std::runtime_error ("Not implemented");
      }
      /// \}

      result_t operator () (const value_type& t) const throw ()
      {
	Configuration_t result (outputSize ());
	impl_compute (result, t);
	if (constraints_)
	  constraints_->apply (result);
	return result;
      }

      void operator () (result_t& result, const value_type& t)
	const throw ()
      {
	impl_compute (result, t);
	if (constraints_)
	  constraints_->apply (result);
      }

      /// \name Constraints
      /// \{

      /// Get constraints the path is subject to
      const ConstraintSetPtr_t& constraints () const
      {
	return constraints_;
      }
      /// Set constraints the path is subject to
      void constraints (const ConstraintSetPtr_t& constraints)
      {
	constraints_ = constraints;
      }
      /// \}
    protected:
      /// Protected constructor
      Path (const interval_t& interval, size_type outputSize) :
	parent_t (interval, outputSize, vector_t ()), constraints_ ()
	{
	}

      Path (const Path& path) : parent_t (path),
	constraints_ (path.constraints_)
	  {
	  }

      /// Store weak pointer to itself
      ///
      /// should be called at construction of derived class instances
      void init (const PathPtr_t& self)
      {
	weak_ = self;
      }
      /// \brief Derivative evaluation.
      ///
      /// Compute the derivative, has to be implemented in concrete classes.
      /// \warning Do not call this function directly, call #derivative instead.
      /// \param derivative derivative will be store in this argument
      /// \param t point where the gradient will be computed
      /// \param order derivative order (if 0 evaluates the function)
      virtual void impl_derivative (gradient_t& derivative,
				    value_type t,
				    size_type order = 1) const throw ();

      /// \brief Derivative evaluation.
      ///
      /// Compute the derivative, has to be implemented in concrete classes.
      /// \warning Do not call this function directly, call #derivative instead.
      /// \param derivative derivative will be store in this argument
      /// \param t point where the gradient will be computed
      /// \param order derivative order (if 0 evaluates the function)
      virtual void
      impl_derivative (gradient_t& g, StableTimePoint_t, size_type order)
	const throw ();

    private:
      /// Constraints that apply to the robot
      ConstraintSetPtr_t constraints_;
      /// Weak pointer to itself
      PathWkPtr weak_;
    }; // class Path
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_HH
